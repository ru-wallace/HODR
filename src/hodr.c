#include "hodr.h"

#define SHUTTER_TYP_OPEN_LOW 0
#define SHUTTER_TYP_OPEN_HIGH 1
#define SHUTTER_MODE_FULLY_AUTO 0

pthread_mutex_t lock;
pthread_mutex_t endThreadLock;
pthread_mutex_t dataFileLock;        // Mutex for data file operations
pthread_mutex_t acquisitionLoopLock; // Mutex for acquisition loop operations
bool endThread = false;              // Flag to signal the command thread to end

uint32_t initialMode = 1;                // Initial acquisition mode
uint32_t initialTargetIntensity = 60000; // Target intensity for the acquisition
uint32_t adjustmentAttemptLimit = 10;    // Limit for adjustment attempts during acquisition
float initialIntegrationTime = 0.01f;    // Initial integration time for the acquisition
float minimumIntegrationTime = 0.00001f; // Minimum integration time for the acquisition
float maximumIntegrationTime = 900.0f;   // Maximum integration time for the acquisition

uint32_t initialPreAmpGainIndex = 0;    // Initial pre-amplifier gain index (0 for 1x, 1 for 1.7x)
int32_t initialTargetTemperature = -70; // Initial target temperature for the acquisition
bool initialCoolerMode = true;          // Initial cooler mode (true for ON, false for OFF)

int32_t initialReadMode = 0;            // Initial read mode for the acquisition
int32_t initialSingleTrackHeight = 20;  // Initial single track height for the acquisition
int32_t initialSingleTrackCentre = 118; // Initial single track centre for the acquisition

bool initialWallclockActive = true; // Initial wallclock acquisition active status
// Must be a factor of 60 to ensure proper timing repeating every hour.
uint32_t initialWallclockInterval = 2; // Initial wallclock acquisition interval in minutes (0 means no schedule)

int mostRecentData[4096];                             // Buffer to hold the most recent data acquired from the detector
bool mostRecentDataValid = false;                     // Flag to indicate if the most recent data is valid
HODR_AcquisitionParams_t mostRecentAcquisitionParams; // Structure to hold acquisition parameters

// char andorFile[256] = "../miniforge3/pkgs/andor2-sdk-2.104.30064-0/etc/andor/";
char initFile[256] = "./hodr.conf"; // Configuration file for HODR
char andorFile[256] = "../andor/etc/";
char outFile[256]; // Output file for data

int countLines();
int createDataFile(char *directory, char *filename);
int appendToFile(float *integrationTime, float *preAmpGain, double *temperature, int *readMode, int *singleTrackHeight, int *singleTrackCentre, const int32_t *data, size_t size);
int readCommandThread(void *arg);
static void dbusOnNameAcquired(GDBusConnection *connection, const gchar *name, gpointer user_data);

static gboolean activateHodr(Control *control);
static gboolean db_activateHodr(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_deactivateHodr(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_resetHodr(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_setTemperature(Control *control, GDBusMethodInvocation *invocation, gint32 value, gpointer user_data);
static gboolean db_getTemperature(gpointer control);
static gboolean db_updateNCaptures(gpointer control);
static gboolean db_setIntegrationTime(Control *control, GDBusMethodInvocation *invocation, gdouble int_time, gpointer user_data);
static gboolean db_setPreAmpGain(Control *control, GDBusMethodInvocation *invocation, gint32 value, gpointer user_data);
static gboolean db_setReadMode(Control *control, GDBusMethodInvocation *invocation, gint32 mode, gpointer user_data);
static gboolean db_setSingleTrack(Control *control, GDBusMethodInvocation *invocation, gint32 centre, gint32 height, gpointer user_data);
static gboolean db_stopLive(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_exitMainLoop(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_startAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_setAcquisitionMode(Control *control, GDBusMethodInvocation *invocation, guint32 mode, gpointer user_data);
static gboolean db_setSeriesLength(Control *control, GDBusMethodInvocation *invocation, guint32 length, gpointer user_data);
static gboolean db_stopAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_setInterval(Control *control, GDBusMethodInvocation *invocation, gdouble interval, gpointer user_data);
static gboolean db_getLastSpectrum(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_setTargetIntensity(Control *control, GDBusMethodInvocation *invocation, guint intensity, gpointer user_data);
static gboolean db_startWallclockAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_stopWallclockAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_setWallclockAcquisitionInterval(Control *control, GDBusMethodInvocation *invocation, guint interval, gpointer user_data);

// static gboolean StartAcquisitionIfPossible(gpointer controlPtr);
static gboolean StartAcquisitionIfPossibleWallclock(gpointer controlPtr);

void setNextAcquisition(int *timeoutID, Control *control); // Function to set the next acquisition
// static gboolean db_getData(Control *control, GDBusMethodInvocation *invocation, gint ref, gpointer user_data);

void *handleAcquisitionLoop();

char dataDir[256] = "../hodr_data"; // Directory for data files

// const float preAmpGainValues[2] = {1.0, 1.7}; // Array of pre-amplifier gain values

double lastTemperature = 0.0;       // Last temperature read from the device
double lastTargetTemperature = 0.0; // Last target temperature set
int lastTemperatureStatus = 0;      // Last temperature status read from the device

uint32_t nTriggeredSpectra = 0; // Number of triggered spectra
uint32_t nCapturedSpectra = 0;  // Number of captured spectra

int xpixels, ypixels; // Detector size
GMainLoop *loop;

guint dataWaitFunctionRef;

// HODR_Config_t *hodr_cfg; // Pointer to HODR configuration structure

bool andorActive = false; // Flag to indicate if Andor SDK is active
bool overHeated = false;  // Flag to indicate if the device is overheated

int timerHandle;

pthread_t acqThread;

void signalHandler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM)
    {
        printf("Received signal %d, exiting...\n", signal);

        fflush(stdout);         // Flush stdout to ensure all output is printed
        endThread = true;       // Set the flag to end the command thread
        AbortAcquisition();     // Abort any ongoing acquisition
        CancelWait();           // Cancel any ongoing wait operations
        g_main_loop_quit(loop); // Quit the main loop
        hodr_deinit();          // Deinitialize HODR
    }
}

int main()
{

    pthread_mutex_init(&lock, NULL);                // Initialize the mutex
    pthread_mutex_init(&endThreadLock, NULL);       // Initialize the end thread mutex
    pthread_mutex_init(&dataFileLock, NULL);        // Initialize the data file mutex
    pthread_mutex_init(&acquisitionLoopLock, NULL); // Initialize the acquisition loop mutex

    int dataFileLength = createDataFile(dataDir, outFile); // Create data file
    if (dataFileLength < 0)
    {
        fprintf(stderr, "Failed to create data file.\n");
        return EXIT_FAILURE; // File creation failed
    }

    signal(SIGTERM, signalHandler); // Register signal handler for SIGINT
    signal(SIGINT, signalHandler);  // Register signal handler for SIGTERM

    printf("HODR initialized successfully.\n");
    printf("Starting D-Bus server...\n");
    loop = g_main_loop_new(NULL, FALSE);
    if (loop == NULL)
    {
        fprintf(stderr, "Failed to create GMainLoop.\n");
        return EXIT_FAILURE; // Loop creation failed
    }

    g_bus_own_name(G_BUS_TYPE_SESSION, "hodr.server.Control", G_BUS_NAME_OWNER_FLAGS_NONE, NULL, dbusOnNameAcquired, NULL, NULL, NULL);
    printf("D-Bus server started successfully.\n");
    printf("Waiting for D-Bus name acquisition...\n");
    g_main_loop_run(loop); // Start the main loop

    printf("Command thread finished.\n");
    CancelWait();

    // pthread_join(acqThread, NULL); // Wait for the acquisition thread to finish
    //  Clean up and shut down the Andor SDK
    AbortAcquisition(); // Abort acquisition if needed

    CoolerOFF(); // Turn off the cooler

    ShutDown();
    printf("Andor SDK shut down successfully.\n");
    return EXIT_SUCCESS;
}

void loadConfig()
{

    if (getConfigValueUnsignedInt(initFile, "mode", &initialMode) == 0)
    {
        printf("Initial acquisition mode read from config: %u\n", initialMode);
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    else
    {
        printf("Failed to read initial acquisition mode from config, using default: %u\n", initialMode);
        fflush(stdout); // Flush stdout to ensure immediate output
    }

    if (getConfigValueInt(initFile, "target_temperature", &initialTargetTemperature) == 0)
    {
        printf("Initial target temperature read from config: %d\n", initialTargetTemperature);
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    else
    {
        printf("Failed to read initial target temperature from config, using default: %d\n", initialTargetTemperature);
        fflush(stdout); // Flush stdout to ensure immediate output
    }

    if (getConfigValueUnsignedInt(initFile, "target_intensity", &initialTargetIntensity) == 0)
    {
        printf("Initial target intensity read from config: %d\n", initialTargetIntensity);
        fflush(stdout); // Flush stdout to ensure immediate output

        if (initialMode == 0)
        {
            initialTargetIntensity = 0; // If mode is 0, set target intensity to 0
            printf("Initial mode is Set Integration Time, setting initial target intensity to 0.\n");
        }
    }
    else
    {
        printf("Failed to read initial target intensity from config, using default: %d\n", initialTargetIntensity);
        fflush(stdout); // Flush stdout to ensure immediate output
    }

    if (getConfigValueFloat(initFile, "integration_time", &initialIntegrationTime) == 0)
    {
        printf("Initial integration time read from config: %f seconds\n", initialIntegrationTime);
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    else
    {
        printf("Failed to read initial integration time from config, using default: %f seconds\n", initialIntegrationTime);
        fflush(stdout); // Flush stdout to ensure immediate output
    }

    if (getConfigValueUnsignedInt(initFile, "pre_amp_gain", &initialPreAmpGainIndex) == 0)
    {
        printf("Initial pre-amp gain read from config: %d\n", initialPreAmpGainIndex);
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    else
    {
        printf("Failed to read initial pre-amp gain from config, using default: %d\n", initialPreAmpGainIndex);
        fflush(stdout); // Flush stdout to ensure immediate output
    }

    if (getConfigValueBool(initFile, "cooler_mode", &initialCoolerMode) == 0)
    {
        printf("Initial cooler mode read from config: %s\n", initialCoolerMode ? "ON" : "OFF");
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    else
    {
        printf("Failed to read initial cooler mode from config, using default: %s\n", initialCoolerMode ? "ON" : "OFF");
        fflush(stdout); // Flush stdout to ensure immediate output
    }

    if (getConfigValueUnsignedInt(initFile, "wallclock_interval_minutes", &initialWallclockInterval) == 0)
    {
        printf("Wallclock acquisition interval read from config: %u minutes\n", initialWallclockInterval);
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    else
    {
        printf("Failed to read wallclock acquisition interval from config, using default: %u minutes\n", initialWallclockInterval);
        fflush(stdout); // Flush stdout to ensure immediate output
    }

    if (getConfigValueBool(initFile, "wallclock_active", &initialWallclockActive) == 0)
    {
        printf("Initial wallclock acquisition active status read from config: %s\n", initialWallclockActive ? "ACTIVE" : "INACTIVE");
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    else
    {
        printf("Failed to read initial wallclock acquisition active status from config, using default: %s\n", initialWallclockActive ? "ACTIVE" : "INACTIVE");
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    if (getConfigValueFloat(initFile, "minimum_integration_time", &minimumIntegrationTime) == 0)
    {
        printf("Minimum integration time read from config: %f seconds\n", minimumIntegrationTime);
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    else
    {
        printf("Failed to read minimum integration time from config, using default: %f seconds\n", minimumIntegrationTime);
        fflush(stdout); // Flush stdout to ensure immediate output
    }

    if (getConfigValueFloat(initFile, "maximum_integration_time", &maximumIntegrationTime) == 0)
    {
        printf("Maximum integration time read from config: %f seconds\n", maximumIntegrationTime);
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    else
    {
        printf("Failed to read maximum integration time from config, using default: %f seconds\n", maximumIntegrationTime);
        fflush(stdout); // Flush stdout to ensure immediate output
    }

    if (getConfigValueUnsignedInt(initFile, "adjustment_attempt_limit", &adjustmentAttemptLimit) == 0)
    {
        printf("Adjustment attempt limit read from config: %u\n", adjustmentAttemptLimit);
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    else
    {
        printf("Failed to read adjustment attempt limit from config, using default: %u\n", adjustmentAttemptLimit);
        fflush(stdout); // Flush stdout to ensure immediate output
    }

    if (getConfigValueInt(initFile, "read_mode", &initialReadMode) == 0)
    {
        printf("Read mode read from config: %u\n", initialReadMode);
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    else
    {
        printf("Failed to read read mode from config, using default: %u\n", initialReadMode);
        fflush(stdout); // Flush stdout to ensure immediate output
    }

    if (getConfigValueInt(initFile, "single_track_height", &initialSingleTrackHeight) == 0)
    {
        printf("Single track height read from config: %u\n", initialSingleTrackHeight);
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    else
    {
        printf("Failed to read single track height from config, using default: %u\n", initialSingleTrackHeight);
        fflush(stdout); // Flush stdout to ensure immediate output
    }

    if (getConfigValueInt(initFile, "single_track_centre", &initialSingleTrackCentre) == 0)
    {
        printf("Single track centre read from config: %u\n", initialSingleTrackCentre);
        fflush(stdout); // Flush stdout to ensure immediate output
    }
    else
    {
        printf("Failed to read single track centre from config, using default: %u\n", initialSingleTrackCentre);
        fflush(stdout); // Flush stdout to ensure immediate output
    }

    printf("Configuration loaded:\n\tmode=%u\n\ttarget_temperature=%d\n\ttarget_intensity=%u\n\tintegration_time=%f\n\tcooler_mode=%s\n\twallclock_interval=%u\n\twallclock_active=%s\n\tread_mode=%u\n\tsingle_track_height=%u\n\tsingle_track_centre=%u\n",
           initialMode,
           initialTargetTemperature,
           initialTargetIntensity,
           initialIntegrationTime,
           initialCoolerMode ? "ON" : "OFF",
           initialWallclockInterval,
           initialWallclockActive ? "ACTIVE" : "INACTIVE",
           initialReadMode,
           initialSingleTrackHeight,
           initialSingleTrackCentre

    );
}

static void dbusOnNameAcquired(GDBusConnection *connection, const gchar *name, gpointer)
{

    printf("D-Bus name '%s' acquired successfully.\n", name);
    Control *control = control_skeleton_new();                                                                        // Create a new Control skeleton
    g_signal_connect(control, "handle-reset", G_CALLBACK(db_resetHodr), NULL);                                        // Connect the signal for resetting HODR
    g_signal_connect(control, "handle-activate", G_CALLBACK(db_activateHodr), NULL);                                  // Connect the signal for activating HODR
    g_signal_connect(control, "handle-deactivate", G_CALLBACK(db_deactivateHodr), NULL);                              // Connect the signal for deactivating HODR
    g_signal_connect(control, "handle-set_temperature", G_CALLBACK(db_setTemperature), NULL);                         // Connect the signal for setting temperature
    g_signal_connect(control, "handle-start_acquisition", G_CALLBACK(db_startAcquisition), NULL);                     // Connect the signal for starting acquisition
    g_signal_connect(control, "handle-set_acquisition_mode", G_CALLBACK(db_setAcquisitionMode), NULL);                // Connect the signal for setting acquisition mode
    g_signal_connect(control, "handle-stop_wallclock_acquisition", G_CALLBACK(db_stopWallclockAcquisition), NULL);    // Connect the signal for stopping wallclock acquisition
    g_signal_connect(control, "handle-set_wallclock_interval", G_CALLBACK(db_setWallclockAcquisitionInterval), NULL); // Connect the signal for setting wallclock acquisition interval
    g_signal_connect(control, "handle-start_wallclock_acquisition", G_CALLBACK(db_startWallclockAcquisition), NULL);  // Connect the signal for starting wallclock acquisition
    g_signal_connect(control, "handle-set_integration_time", G_CALLBACK(db_setIntegrationTime), NULL);                // Connect the signal for setting integration time
    g_signal_connect(control, "handle-set_pre_amp_gain", G_CALLBACK(db_setPreAmpGain), NULL);                         // Connect the signal for setting preamp gain
    g_signal_connect(control, "handle-set_read_mode", G_CALLBACK(db_setReadMode), NULL);                              // Connect the signal for setting read mode
    g_signal_connect(control, "handle-set_single_track", G_CALLBACK(db_setSingleTrack), NULL);                        // Connect the signal for setting single track parameters
    g_signal_connect(control, "handle-stop_acquisition", G_CALLBACK(db_stopAcquisition), NULL);                       // Connect the signal for stopping acquisition
    g_signal_connect(control, "handle-set_target_intensity", G_CALLBACK(db_setTargetIntensity), NULL);                // Connect the signal for setting target intensity
    g_signal_connect(control, "handle-set_interval", G_CALLBACK(db_setInterval), NULL);                               // Connect the signal for setting interval
    g_signal_connect(control, "handle-set_series_length", G_CALLBACK(db_setSeriesLength), NULL);                      // Connect the signal for setting series length
    g_signal_connect(control, "handle-stop_live", G_CALLBACK(db_stopLive), NULL);                                     // Connect the signal for stopping live mode
    g_signal_connect(control, "handle-get_data", G_CALLBACK(db_getLastSpectrum), NULL);                               // Connect the signal for getting data
    g_signal_connect(control, "handle-exit", G_CALLBACK(db_exitMainLoop), NULL);                                      // Connect the signal for exiting the application

    int nSpectra = countLines();

    loadConfig(); // Load configuration from the config file

    printf("Number of spectra in data file: %d\n", nSpectra);
    fflush(stdout); // Flush stdout to ensure immediate output
    if (nSpectra >= 0)
        nCapturedSpectra = (uint32_t)nSpectra;
    control_set_number_spectra(control, nCapturedSpectra); // Initialize number of spectra to 0
    control_set_data_path(control, outFile);               // Set the data path in the control object
    control_set_wallclock_next_capture(control, 0);        // Initialize wallclock next capture to 0

    control_set_over_heat_reboot_count(control, 0); // Initialize overheat reboot count to 0

    gboolean preAmpGainSet = initialPreAmpGainIndex == 1;

    control_set_pre_amp_gain(control, preAmpGainSet); // Set the initial pre-amp gain in the control object

    control_set_temperature(control, initialTargetTemperature); // Set the initial target temperature in the control object

    control_set_target_intensity(control, initialTargetIntensity);             // Set the target intensity in the control object
    control_set_target_temperature(control, initialTargetTemperature);         // Set the target temperature in the control object
    control_set_wallclock_interval(control, initialWallclockInterval);         // Set wallclock acquisition interval in the control object
    control_set_wallclock_acquisition_active(control, initialWallclockActive); // Set wallclock acquisition active status in the control object

    activateHodr(control); // Activate HODR

    g_timeout_add_seconds(1, db_updateNCaptures, control);                                                          // Schedule next update of number of captures
    g_dbus_interface_skeleton_export(G_DBUS_INTERFACE_SKELETON(control), connection, "/hodr/server/Control", NULL); // Export the control interface on D-Bus
}

static gboolean powerOnHodr(Control *control)
{
    if (control == NULL)
    {
        fprintf(stderr, "Control object is NULL.\n");
        fflush(stderr); // Flush stderr to ensure immediate output
        return FALSE;   // Handle null control object
    }

    if (control_get_over_heat_reboot_count(control) > 2)
    {
        printf("Overheat reboot count is greater than 2, skipping power on.\n");
        fflush(stdout); // Flush stdout to ensure immediate output
        return FALSE;   // Skip power on if overheat reboot count is greater than 2
    }

    // Emit power on signal
    control_emit_power_on(control, 1);
    sleep(1);                          // Sleep for 1 second to allow the signal to be processed
    control_emit_power_on(control, 1); // Emit power on signal with reason 0
    return FALSE;
}

static gboolean activateHodr(Control *control)
{

    if (andorActive) // Check if Andor SDK is already active
    {
        printf("HODR is already active.\n");
        fflush(stdout); // Flush stdout to ensure immediate output
        return FALSE;   // HODR is already active
    }

    if (control == NULL)
    {
        fprintf(stderr, "Control object is NULL.\n");
        fflush(stderr); // Flush stderr to ensure immediate output
        return FALSE;   // Handle null control object
    }

    if (control_get_over_heat_reboot_count(control) > 2)
    {
        printf("Overheat reboot count is greater than 2, skipping activation.\n");
        fflush(stdout); // Flush stdout to ensure immediate output
        return FALSE;   // Skip activation if overheat reboot count is greater than 2
    }

    pthread_mutex_lock(&lock);                                  // Lock the mutex to ensure thread safety
    unsigned int result = hodr_init(andorFile, outFile, false); // Initialize HODR
    if (result == DRV_SUCCESS)
    {
        control_emit_power_on(control, 1); // Emit power on signal with reason 1
        andorActive = true;                // Set Andor SDK active flag to TRUE
        control_set_active(control, TRUE); // Set the control object as active
        control_set_live(control, TRUE);   // Initialize live status to TRUE

        printf("HODR activated successfully.\n");
        hodr_setCoolerMode(true); // Turn on the cooler

        control_set_over_heat_count(control, 0);                    // Set the overheat count in the control object
        control_set_over_heat_cooldown_active(control, FALSE);      // Set cooldown active in the control object
        float targetTemp = control_get_target_temperature(control); // Get target temperature from control object
        hodr_setTargetTemperature(targetTemp);                      // Set target temperature in HODR
        printf("Target temperature set to: %.2f\n", targetTemp);
        fflush(stdout);                                                    // Flush stdout to ensure immediate output
        g_timeout_add_seconds(1, (GSourceFunc)db_getTemperature, control); // Schedule next temperature check
        hodr_getDetectorSize(&xpixels, &ypixels);                          // Get detector size
        printf("Detector size: %d x %d pixels\n", xpixels, ypixels);

        hodr_setPreAmpGain(control_get_pre_amp_gain(control) ? 1 : 0); // Set pre-amp gain based on control object

        int preAmpGainIndex;
        char gainString[32];
        hodr_getCurrentPreAmpGain(&preAmpGainIndex, gainString);
        control_set_pre_amp_gain(control, preAmpGainIndex == 1); // Set the pre-amp gain in the control object

        float integrationTime, accumulate, kinetic;

        result = GetAcquisitionTimings(&integrationTime, &accumulate, &kinetic);
        if (result != DRV_SUCCESS)
        {
            fprintf(stderr, "Failed to get acquisition timings: %d\n", result);
        }
        else
        {
            printf("Current Acquisition Timings - Exposure: %.6f s, Accumulate: %.6f s, Kinetic: %.6f s\n", integrationTime, accumulate, kinetic);
            control_set_integration_time_secs(control, integrationTime); // Set the integration time in the control object
        }

        hodr_setReadMode(initialReadMode);                                       // Set read mode to Full Vertical Binning
        hodr_setSingleTrack(initialSingleTrackCentre, initialSingleTrackHeight); // Set single track parameters

        control_set_read_mode(control, initialReadMode);                    // Set the read mode in the control object
        control_set_single_track_centre(control, initialSingleTrackCentre); // Set the single track centre in the control object
        control_set_single_track_height(control, initialSingleTrackHeight); // Set the single track height in the control object

        int numberPreAmpGains = 0;
        GetNumberPreAmpGains(&numberPreAmpGains);
        printf("Number of PreAmp Gains: %d\n", numberPreAmpGains);

        printf("Current PreAmp Gain: %s (Index: %d)\n", gainString, preAmpGainIndex);

        fflush(stdout); // Flush stdout to ensure immediate output

        if (control_get_wallclock_acquisition_active(control)) // Check if wallclock acquisition is active
        {
            printf("Wallclock acquisition is active. Setting next acquisition...\n");
            setNextAcquisition(&timerHandle, control); // Set the next acquisition
        }
        else
        {
            printf("Wallclock acquisition is not active.\n");
        }

        int status;
        hodr_getStatus(&status);                         // Get the current acquisition status from HOD
        control_set_acquisition_status(control, status); // Set the acquisition status in the control object

        pthread_create(&acqThread, NULL, handleAcquisitionLoop, control); // Create a thread for handling acquisition loop

        pthread_mutex_unlock(&lock); // Unlock the mutex after activating HODR
        return FALSE;                // Returning false ensures timout function does not get called again
    }
    else
    {
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        fprintf(stderr, "Failed to activate HODR: %d\n", result);
        return FALSE; // Error activating HODR
    }
}
static gboolean db_activateHodr(Control *control, GDBusMethodInvocation *invocation, gpointer)
{
    if (andorActive) // Check if Andor SDK is already active
    {
        control_complete_activate(control, invocation, TRUE); // Complete the D-Bus method invocation with success
        control_set_active(control, TRUE);                    // Set the control object as active
        return TRUE;                                          // HODR is already active
    }

    printf("Activating HODR from DBUS call...\n");
    fflush(stdout);                                       // Flush stdout to ensure immediate output
    activateHodr(control);                                // Activate HODR
    control_complete_activate(control, invocation, TRUE); // Complete the D-Bus method invocation

    return TRUE;
}

static gboolean db_deactivateHodr(Control *control, GDBusMethodInvocation *invocation, gpointer)
{

    if (!andorActive) // Check if Andor SDK is not active
    {
        control_complete_deactivate(control, invocation, TRUE); // Complete the D-Bus method invocation with success
        control_set_active(control, FALSE);                     // Set the control object as inactive
        return TRUE;                                            // HODR is not active
    }
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    printf("Deactivating HODR...\n");
    CancelWait();       // Cancel any ongoing wait operations
    AbortAcquisition(); // Abort any ongoing acquisition

    int status;
    hodr_getStatus(&status);                         // Get the current acquisition status from HOD
    control_set_acquisition_status(control, status); // Set the acquisition status in the control object

    g_source_remove(timerHandle);        // Remove the timer for wallclock acquisition
    unsigned int result = hodr_deinit(); // Deinitialize HODR
    if (result != DRV_SUCCESS)
    {
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_FAILED, "Failed to deactivate HODR: %d", result); // Return error response
        pthread_mutex_unlock(&lock);                                                                                               // Unlock the mutex before returning
        return FALSE;                                                                                                              // Error deactivating HODR
    }
    andorActive = false;                                    // Set Andor SDK active flag to FALSE
    control_complete_deactivate(control, invocation, TRUE); // Complete the D-Bus method invocation
    control_set_active(control, FALSE);                     // Set the control object as inactive
    control_set_temperature_status(control, "DEACTIVATED"); // Set temperature status to DEACTIVATED
    pthread_mutex_unlock(&lock);                            // Unlock the mutex after deactivating HODR
    printf("HODR deactivated successfully.\n");
    return TRUE; // Successfully deactivated HODR
}

static gboolean db_resetHodr(Control *control, GDBusMethodInvocation *invocation, gpointer)
{
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    printf("Resetting HODR...\n");
    if (andorActive)
    {
        CancelWait();                        // Cancel any ongoing wait operations
        AbortAcquisition();                  // Abort any ongoing acquisition
        unsigned int result = hodr_deinit(); // Deinitialize HODR
        if (result != DRV_SUCCESS)
        {
            control_complete_reset(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
            pthread_mutex_unlock(&lock);                        // Unlock the mutex before returning
            return FALSE;                                       // Error resetting HODR
        }
        andorActive = false;                // Set Andor SDK active flag to FALSE
        control_set_active(control, FALSE); // Set the control object as inactive
    }

    unsigned int initResult = hodr_init(andorFile, outFile, true); // Reinitialize HODR
    if (initResult != DRV_SUCCESS)
    {
        control_complete_reset(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        return FALSE;                                       // Error resetting HODR
    }
    andorActive = true;                                            // Set Andor SDK active flag to TRUE
    hodr_setCoolerMode(true);                                      // Turn on the cooler
    pthread_create(&acqThread, NULL, handleAcquisitionLoop, NULL); // Create a thread for handling acquisition loop

    pthread_mutex_unlock(&lock);                       // Unlock the mutex after resetting HODR
    control_complete_reset(control, invocation, TRUE); // Complete the D-Bus method invocation with success
    control_set_active(control, TRUE);                 // Set the control object as active
    return TRUE;                                       // Successfully reset HODR
}

static gboolean db_exitMainLoop(Control *control, GDBusMethodInvocation *invocation, gpointer)
{
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    printf("Exiting main loop...\n");
    g_main_loop_quit(loop); // Quit the main loop
    // g_dbus_method_invocation_return_value(invocation, NULL); // Return success response
    control_complete_exit(control, invocation); // Complete the D-Bus method invocation
    pthread_mutex_unlock(&lock);                // Unlock the mutex after quitting the loops
    return TRUE;                                // Successfully exited main loop
}

static gboolean db_stopLive(Control *control, GDBusMethodInvocation *invocation, gpointer)
{
    gboolean live = control_get_live(control); // Get live status from the control object
    if (live)
    {
        control_set_live(control, FALSE);                        // Set live status to FALSE
        g_dbus_method_invocation_return_value(invocation, NULL); // Return success response
        printf("Live mode stopped successfully.\n");
    }
    else
    {
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_FAILED, "Live mode is not active."); // Return error if live mode is not active
        printf("Live mode was not active.\n");
    }
    return TRUE; // Successfully stopped live mode
}

bool captureCountPrinted = false; // Flag to indicate if the capture count has been printed already since SDK became inactive

static gboolean db_updateNCaptures(gpointer control)
{
    if (!andorActive) // Check if Andor SDK is active
    {
        if (!captureCountPrinted) // Check if the capture count has already been printed
        {
            printf("Andor SDK is not active. Skipping update of number of captures.\n");
            fflush(stdout);             // Flush stdout to ensure immediate output
            captureCountPrinted = true; // Set the flag to indicate that the message has been printed
        }

        return TRUE; // Do not update if Andor SDK is not active
    }
    captureCountPrinted = false; // Reset the flag if Andor SDK is active

    if (control == NULL)
    {
        fprintf(stderr, "Control object is NULL.\n");
        fflush(stderr); // Flush stderr to ensure immediate output
        return FALSE;   // Handle null control object
    }

    if (control_get_over_heat_reboot_count(control) > 2)
    {
        printf("Overheat reboot count is greater than 2, stopping updates.\n");
        fflush(stdout); // Flush stdout to ensure immediate output
        return FALSE;   // Skip update if overheat reboot count is greater than 2
    }

    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    int status;
    unsigned int result = hodr_getStatus(&status); // Get the current status of HODR
    if (result == DRV_SUCCESS)
    {
        control_set_acquisition_status(control, status); // Set the acquisition status in the control object
    }

    if (control_get_number_spectra(control) != nCapturedSpectra)
    {
        printf("Updating number of captures: %d\n", nCapturedSpectra);
    }
    control_set_number_spectra(control, nCapturedSpectra); // Update the number of captured spectra in the control object

    pthread_mutex_unlock(&lock); // Unlock the mutex after updating
    return TRUE;                 // Successfully updated number of captures
}

static gboolean db_setIntegrationTime(Control *control, GDBusMethodInvocation *invocation, gdouble int_time, gpointer)
{
    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not setting integration time.\n");
        control_complete_set_integration_time(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        return TRUE;                                                       // Do not update if Andor SDK is not active
    }
    printf("Setting integration time to %.9f seconds...\n", int_time);
    printf("Waiting for mutex lock to set integration time...\n");
    fflush(stdout);            // Flush stdout to ensure immediate output
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    printf("Mutex lock acquired for setting integration time.\n");
    fflush(stdout); // Flush stdout to ensure immediate output

    if (int_time < 0)
    {
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT, "Invalid integration time: %.9f", int_time);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return TRUE;                 // Invalid integration time
    }

    control_set_target_intensity(control, 0);             // Set target intensity to 0 before changing integration time
    unsigned int result = hodr_setExposureTime(int_time); // Set exposure time in HODR
    if (result != DRV_SUCCESS)
    {

        result = hodr_changeExposureTimeDuringSeries(int_time, NULL); // Attempt to change exposure time during series
        if (result != DRV_SUCCESS)
        {

            control_complete_set_integration_time(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
            printf("Failed to set integration time: %d\n", result);
            pthread_mutex_unlock(&lock); // Unlock the mutex before returning
            return TRUE;                 // Error setting integration time
        }
    }

    float currentIntegrationTime;
    float accumulateTime;
    float kineticTime;
    hodr_getAcquisitionTimings(&currentIntegrationTime, &accumulateTime, &kineticTime); // Get current acquisition timings

    printf("Current integration time after setting: %.9f seconds\n", currentIntegrationTime);

    control_set_integration_time_secs(control, currentIntegrationTime); // Update integration time in control object
    fflush(stdout);                                                     // Flush stdout to ensure immediate output
    control_complete_set_integration_time(control, invocation, TRUE);   // Complete the D-Bus method invocation with success
    printf("Integration time set to %.9f seconds successfully.\n", currentIntegrationTime);
    pthread_mutex_unlock(&lock); // Unlock the mutex after setting the integration time
    return TRUE;                 // Successfully set integration time
}

static gboolean db_setReadMode(Control *control, GDBusMethodInvocation *invocation, gint32 mode, gpointer)
{

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not setting read mode.\n");
        control_complete_set_read_mode(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        return FALSE;                                               // Do not update if Andor SDK is not active
    }
    printf("Setting read mode to %d...\n", mode);
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    unsigned int result = hodr_setReadMode(mode); // Set read mode in HODR

    if (result != DRV_SUCCESS)
    {
        control_complete_set_read_mode(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        printf("Failed to set read mode: %d\n", result);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return FALSE;                // Error setting read mode
    }

    control_set_read_mode(control, mode);                      // Update read mode in control object
    control_complete_set_read_mode(control, invocation, TRUE); // Complete the D-Bus method invocation with success
    printf("Read mode set to %d successfully.\n", mode);
    pthread_mutex_unlock(&lock); // Unlock the mutex after setting the read mode
    return TRUE;                 // Successfully set read mode
}

static gboolean db_setSingleTrack(Control *control, GDBusMethodInvocation *invocation, gint32 centre, gint32 height, gpointer)
{

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not setting single track.\n");
        control_complete_set_single_track(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        return FALSE;                                                  // Do not update if Andor SDK is not active
    }
    printf("Setting single track centre: %d and height: %d...\n", centre, height);
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    unsigned int result = hodr_setSingleTrack((int)centre, (int)height); // Set single track in HODR
    if (result != DRV_SUCCESS)
    {
        control_complete_set_single_track(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        printf("Failed to set single track: %d\n", result);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return FALSE;                // Error setting single track
    }

    control_set_single_track_centre(control, centre); // Update single track centre in control object
    control_set_single_track_height(control, height); // Update single track height in control object

    control_complete_set_single_track(control, invocation, TRUE); // Complete the D-Bus method invocation with success
    printf("Single track set to centre: %d and height: %d successfully.\n", centre, height);
    pthread_mutex_unlock(&lock); // Unlock the mutex after setting the single track
    return TRUE;                 // Successfully set single track
}

static gboolean db_setPreAmpGain(Control *control, GDBusMethodInvocation *invocation, gboolean value, gpointer)
{

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not setting preamp gain.\n");
        control_complete_set_pre_amp_gain(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        return FALSE;                                                  // Do not update if Andor SDK is not active
    }
    printf("Setting preamp gain to index %d...\n", value);
    fflush(stdout);            // Flush stdout to ensure immediate output
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    unsigned int result = hodr_setPreAmpGain((int)value); // Set preamp gain in HODR
    if (result != DRV_SUCCESS)
    {
        control_complete_set_pre_amp_gain(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        printf("Failed to set preamp gain: %d\n", result);
        fflush(stdout);              // Flush stdout to ensure immediate output
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return FALSE;                // Error setting preamp gain
    }
    control_set_pre_amp_gain(control, value);                      // Update preamp gain in control object
    control_complete_set_pre_amp_gain(control, invocation, TRUE); // Complete the D-Bus method invocation with success
    printf("Preamp gain set to index %d successfully.\n", value);
    fflush(stdout);              // Flush stdout to ensure immediate output
    pthread_mutex_unlock(&lock); // Unlock the mutex after setting the preamp gain
    return TRUE;                 // Successfully set preamp gain
}

static gboolean db_setInterval(Control *control, GDBusMethodInvocation *invocation, gdouble interval, gpointer)
{

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not setting interval.\n");
        control_complete_set_interval(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        return FALSE;                                              // Do not update if Andor SDK is not active
    }
    printf("Setting interval to %.5f seconds...\n", (float)interval);
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    if (interval <= 0)
    {
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT, "Invalid interval: %.5f", (float)interval);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return FALSE;                // Invalid interval
    }

    unsigned int result = hodr_setKineticCycleTime((float)interval); // Set kinetic cycle time in HODR
    control_set_interval_time_secs(control, interval);               // Update interval in control object
    if (result != DRV_SUCCESS)
    {
        control_complete_set_interval(control, invocation, FALSE); // Complete the D-Bus method invocation
        printf("Failed to set interval: %d\n", result);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return FALSE;                // Error setting interval
    }
    control_complete_set_interval(control, invocation, TRUE); // Complete the D-Bus method invocation
    printf("Interval set to %.5f seconds successfully.\n", (float)interval);
    pthread_mutex_unlock(&lock); // Unlock the mutex after setting the interval

    int status;
    hodr_getStatus(&status);                         // Get the current acquisition status from HOD
    control_set_acquisition_status(control, status); // Set the acquisition status in the control object

    return TRUE; // Successfully set interval
}

static gboolean db_setSeriesLength(Control *control, GDBusMethodInvocation *invocation, guint32 length, gpointer)
{

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not setting series length.\n");
        control_complete_set_series_length(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        return FALSE;                                                   // Do not update if Andor SDK is not active
    }
    printf("Setting series length to %d...\n", length);
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    if (length <= 0)
    {
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT, "Invalid series length: %d", length);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return FALSE;                // Invalid series length
    }

    hodr_setNumberKinetics((int)length); // Set number of kinetics in HODR

    control_set_series_length(control, length);                    // Update series length in control object
    control_complete_set_series_length(control, invocation, TRUE); // Complete the D-Bus method invocation
    printf("Series length set to %d successfully.\n", length);
    pthread_mutex_unlock(&lock); // Unlock the mutex after setting the series length

    return TRUE; // Successfully set series length
}

static gboolean db_setAcquisitionMode(Control *control, GDBusMethodInvocation *invocation, guint32 mode, gpointer)
{

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not setting acquisition mode.\n");
        control_complete_set_acquisition_mode(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        return FALSE;                                                      // Do not update if Andor SDK is not active
    }

    printf("Setting acquisition mode to %d...\n", mode);
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    if (mode > 5)              // Assuming valid modes are 0, 1, and 2
    {
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT, "Invalid acquisition mode: %d", mode);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return FALSE;                // Invalid acquisition mode
    }

    hodr_setAcquisitionMode((int)mode);          // Set the acquisition mode in HODR
    control_set_acquisition_mode(control, mode); // Update acquisition mode in control object

    control_complete_set_acquisition_mode(control, invocation, TRUE); // Set the acquisition mode in the control object

    printf("Acquisition mode set to %d successfully.\n", mode);
    pthread_mutex_unlock(&lock); // Unlock the mutex after setting acquisition mode
    return TRUE;                 // Successfully set acquisition mode
}

static gboolean db_setTargetIntensity(Control *control, GDBusMethodInvocation *invocation, guint intensity, gpointer)
{

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not setting target intensity.\n");
        fflush(stdout);                                                    // Flush stdout to ensure immediate output
        control_complete_set_target_intensity(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        return TRUE;                                                       // Do not update if Andor SDK is not active
    }

    printf("Setting target intensity to %u...\n", intensity);
    printf("Waiting to acquire lock for setting target intensity...\n");
    fflush(stdout);            // Flush stdout to ensure immediate output
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    printf("Acquired lock for setting target intensity.\n");
    fflush(stdout); // Flush stdout to ensure immediate output

    control_set_target_intensity(control, intensity); // Set the target intensity in the control object
    printf("Target intensity set to %u in control object.\n", intensity);
    fflush(stdout);                                                   // Flush stdout to ensure immediate output
    control_complete_set_target_intensity(control, invocation, TRUE); // Complete the D-Bus method invocation with success
    printf("Target intensity set to %d successfully.\n", intensity);
    fflush(stdout);              // Flush stdout to ensure immediate output
    pthread_mutex_unlock(&lock); // Unlock the mutex after setting target intensity
    return TRUE;                 // Successfully set target intensity
}

bool temperatureReported = false; // Flag to indicate if temperature has been reported while Andor SDK is inactive

static gboolean db_getTemperature(gpointer control)
{

    if (control == NULL)
    {
        fprintf(stderr, "Control object is NULL.\n");
        fflush(stderr); // Flush stderr to ensure immediate output
        return FALSE;   // Handle null control object
    }

    if (control_get_over_heat_reboot_count(control) > 2)
    {
        printf("Overheat reboot count is greater than 2, ending temperature monitoring.\n");
        fflush(stdout); // Flush stdout to ensure immediate output
        return FALSE;   // Skip temperature update if overheat reboot count is greater than 2
    }

    if (!andorActive) // Check if Andor SDK is active
    {
        if (!temperatureReported) // Check if temperature has already been reported while Andor SDK is inactive
        {
            printf("Andor SDK is not active. Not getting temperature.\n");
            fflush(stdout);             // Flush stdout to ensure immediate output
            temperatureReported = true; // Set the flag to indicate that the message has been printed
        }
        return TRUE; // Do not update if Andor SDK is not active
    }
    temperatureReported = false; // Reset the flag since Andor SDK is active

    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    float currentTemp, targetTemp;

    unsigned int result = hodr_getCurrentTemperatureAndTargetTemperature(&currentTemp, &targetTemp); // Get current and target temperatures

    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to get current temperature: %d\n", result);
        fflush(stdout);              // Flush stdout to ensure immediate output
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return TRUE;                 // Error getting temperature
    }

    double currentTempDouble = (double)currentTemp; // Convert current temperature to double
    double targetTempDouble = (double)targetTemp;   // Convert target temperature to double

    unsigned int overHeatCount = control_get_over_heat_count(control); // Get the current overheat count from the control object

    if (currentTempDouble >= 25 && targetTempDouble < 20)
    {
        printf("Warning: Current temperature (%.2f) is above 25 and target temperature (%.2f) is below 20.\n", currentTempDouble, targetTempDouble);
        fflush(stdout);                                        // Flush stdout to ensure immediate output
        control_set_over_heat_count(control, ++overHeatCount); // Increment overheat count in the control object
    }
    else
    {
        if (overHeatCount > 0)
        {
            printf("Overheating condition resolved. Overheat count reset.\n");
            fflush(stdout); // Flush stdout to ensure immediate output
        }
        control_set_over_heat_count(control, 0); // Reset overheat count if condition is resolved
    }

    if (control_get_over_heat_count(control) >= 30) // Check if overheating condition has occurred 3 times
    {

        control_set_over_heat_cooldown_active(control, TRUE); // Activate overheat cooldown in the control object
        control_set_over_heat_count(control, 0);              // Reset overheat count in the control object
        time_t now;
        time(&now);                             // Get the current time
        struct tm *localTime = localtime(&now); // Convert to local time

        printf("Overheating condition detected at %02d:%02d:%02d. Shutting down HODR.\n", localTime->tm_hour, localTime->tm_min, localTime->tm_sec);
        fflush(stdout); // Flush stdout to ensure immediate output

        time_t endTime = now + 300;                    // Set end time for cooldown (5 minutes later)
        struct tm *endLocalTime = localtime(&endTime); // Convert to local time

        printf("Cooldown period until %02d:%02d:%02d.\n", endLocalTime->tm_hour, endLocalTime->tm_min, endLocalTime->tm_sec);
        control_set_over_heat_cooldown_active(control, TRUE);             // Set cooldown active in the control object
        control_set_over_heat_cooldown_end_time(control, (guint)endTime); // Set cooldown end time in the control object

        printf("Overheating condition detected. Shutting down HODR.\n");
        fflush(stdout);                     // Flush stdout to ensure immediate output
        overHeated = true;                  // Set overheating flag
        CancelWait();                       // Cancel any ongoing wait operations
        AbortAcquisition();                 // Abort any ongoing acquisition
        hodr_deinit();                      // Deinitialize HODR
        andorActive = false;                // Set Andor SDK active flag to FALSE
        pthread_mutex_unlock(&lock);        // Unlock the mutex before returning
        control_emit_power_off(control, 1); // Emit power off signal
        // Add 5 min timeout to reinitialize HODR

        if (control_get_over_heat_reboot_count(control) < 3)
        {
            control_set_over_heat_reboot_count(control, control_get_over_heat_reboot_count(control) + 1); // Increment overheat reboot count
        }
        else
        {
            printf("Overheat reboot count exceeded. Not scheduling power on.\n");
            fflush(stdout); // Flush stdout to ensure immediate output

            endThread = true; // Set the flag to end the command thread

            g_main_loop_quit(loop); // Quit the main loop

            return FALSE; // Do not schedule power on if overheat reboot count exceeded
        }

        g_timeout_add_seconds(280, (GSourceFunc)powerOnHodr, control); // Schedule power on after 5 minutes
        g_timeout_add_seconds(300, (GSourceFunc)activateHodr, control);

        return FALSE; // Stop further temperature updates
    }

    int tempStatus;
    hodr_getCurrentTemperatureStatus(&tempStatus); // Get current temperature status

    int status;
    hodr_getStatus(&status);                         // Get the current acquisition status from HOD
    control_set_acquisition_status(control, status); // Set the acquisition status in the control object

    if (currentTempDouble != lastTemperature ||
        targetTempDouble != lastTargetTemperature || tempStatus != lastTemperatureStatus) // Check if temperatures or status have changed
    {
        char tempStatusString[64];                                         // Buffer for temperature status string
        hodr_getTemperatureStatusString(tempStatus, tempStatusString, 64); // Get temperature status string
        printf("Current Temperature: %.2f, Target Temperature: %.2f, Status: %s\n", currentTempDouble, targetTempDouble, tempStatusString);
        fflush(stdout);                           // Flush stdout to ensure immediate output
        lastTargetTemperature = targetTempDouble; // Update last target temperature
        lastTemperature = currentTempDouble;      // Update last temperature
        lastTemperatureStatus = tempStatus;       // Update last temperature status
        fflush(stdout);                           // Flush stdout to ensure immediate output

        control_set_target_temperature(control, targetTempDouble); // Set target temperature in the control object
        control_set_temperature(control, currentTempDouble);       // Set current temperature in the control object
        control_set_temperature_status(control, tempStatusString); // Set temperature status in the control object
    }

    pthread_mutex_unlock(&lock); // Unlock the mutex after getting temperature
    // control_emit_temperature_status(control, currentTempDouble, tempStatusString, targetTempDouble); // Emit temperature status signal

    gboolean live = control_get_live(control); // Get live status from the control object

    if (!live)
    {
        printf("Live mode is not active. Stopping temperature updates.\n");
        return FALSE;
    }

    return TRUE; // Successfully got temperature
}

static gboolean db_setTemperature(Control *control, GDBusMethodInvocation *invocation, gint32 value, gpointer)
{

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not setting temperature.\n");
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_FAILED, "Andor SDK is not active.");
        return FALSE; // Do not update if Andor SDK is not active
    }

    // get lock to ensure thread safety
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    // cast unsigned guint to int

    if (value < -120 || value > 20)
    { // Check if the temperature is within a valid range
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT, "Temperature out of range: %d", value);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return FALSE;                // Invalid temperature
    }

    if (value == -13)
    {
        andorActive = false; // Set Andor SDK active flag to FALSE
        printf("Setting temperature to -13 degrees Celsius. Emitting power off signal.\n");
        fflush(stdout); // Flush stdout to ensure immediate output
        CancelWait();   // Cancel any ongoing wait operations
        // printf("Deinitializing HODR before power off.\n");
        // fflush(stdout); // Flush stdout to ensure immediate output
        // hodr_deinit();           // Deinitialize HODR

        printf("Emitting power off signal.\n");
        fflush(stdout);                     // Flush stdout to ensure immediate output
        control_emit_power_off(control, 0); // Emit power off signal
        sleep(1);                           // Sleep for 1 second to repeat power off signal in case of missed signal
        control_emit_power_off(control, 0); // Emit power off signal with reason 0
        printf("Scheduling power on and activation after 10 seconds.\n");
        fflush(stdout);                                                // Flush stdout to ensure immediate output
        g_timeout_add_seconds(10, (GSourceFunc)powerOnHodr, control);  // Schedule power on after 10 seconds
        g_timeout_add_seconds(20, (GSourceFunc)activateHodr, control); // Schedule activation after 20 seconds
        control_set_active(control, FALSE);                            // Set the control object to inactive
        hodr_setTargetTemperature(value);                              // Set the target temperature in HODR
        pthread_mutex_unlock(&lock);                                   // Unlock the mutex before returning

        return TRUE; // Successfully set temperature
    }

    hodr_setTargetTemperature(value); // Set the target temperature in HODR

    float currentTemp, targetTemp;
    hodr_getCurrentTemperatureAndTargetTemperature(&currentTemp, &targetTemp); // Get current and target temperatures
    hodr_getCurrentTemperatureStatus(&value);                                  // Get current temperature status
    char tempStatusString[64];                                                 // Buffer for temperature status string
    hodr_getTemperatureStatusString(value, tempStatusString, 64);              // Get temperature status string
    control_set_target_temperature(control, targetTemp);                       // Set the target temperature in the control object
    control_set_temperature(control, currentTemp);                             // Set the current temperature in the control object
    control_set_temperature_status(control, tempStatusString);                 // Set the temperature status in the control object

    control_complete_set_temperature(control, invocation, TRUE); // Complete the D-Bus method invocation
    printf("Target Temperature set to %.2f successfully.\n", targetTemp);
    pthread_mutex_unlock(&lock); // Unlock the mutex after setting temperature
    return TRUE;                 // Successfully set temperature
}

// static gboolean StartAcquisitionIfPossible(gpointer controlPtr)
// {
//     Control *control = (Control *)controlPtr;

//     printf("Attempting to start acquisition...\n");
//     fflush(stdout);   // Flush stdout to ensure immediate output
//     if (!andorActive) // Check if Andor SDK is active
//     {
//         printf("Andor SDK is not active. Not starting acquisition.\n");

//         return false; // Do not start acquisition if Andor SDK is not active
//     }

//     pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

//     hodr_setAcquisitionMode(AC_ACQMODE_SINGLE);               // Set acquisition mode to 1 (assuming this is a valid mode)
//     control_set_acquisition_mode(control, AC_ACQMODE_SINGLE); // Set the acquisition mode in the control object

//     unsigned int result = hodr_startAcquisition(); // Start acquisition in HODR

//     int status;
//     hodr_getStatus(&status);                         // Get the current acquisition status from HOD
//     control_set_acquisition_status(control, status); // Set the acquisition status in the control object

//     pthread_mutex_unlock(&lock); // Unlock the mutex after starting acquisition
//     if (result != DRV_SUCCESS)
//     {
//         fprintf(stderr, "Failed to start acquisition: %d\n", result);
//     }
//     else
//     {
//         printf("Acquisition started successfully.\n");
//         fflush(stdout); // Flush stdout to ensure immediate output

//         nTriggeredSpectra++;
//         return true; // Successfully started acquisition
//     }

//     return false; // Failed to start acquisition
// }

static gboolean StartAcquisitionIfPossibleWallclock(gpointer controlPtr)
{
    Control *control = (Control *)controlPtr;

    printf("Attempting to start acquisition in wallclock mode...\n");
    fflush(stdout);   // Flush stdout to ensure immediate output
    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not starting acquisition.\n");

        return false; // Do not start acquisition if Andor SDK is not active
    }

    if (!control_get_wallclock_acquisition_active(control)) // Check if wallclock acquisition is active
    {
        printf("Wallclock acquisition is not active. Not starting acquisition.\n");
        fflush(stdout); // Flush stdout to ensure immediate output
        return false;   // Do not start acquisition if wallclock acquisition is not active
    }

    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    hodr_setAcquisitionMode(AC_ACQMODE_SINGLE);               // Set acquisition mode to 1 (assuming this is a valid mode)
    control_set_acquisition_mode(control, AC_ACQMODE_SINGLE); // Set the acquisition mode in the control object

    // g_source_remove(timerHandle);                 // Remove the existing timer to prevent multiple acquisitions

    unsigned int result = hodr_startAcquisition(); // Start acquisition in HODR

    int status;
    hodr_getStatus(&status);                         // Get the current acquisition status from HODR
    control_set_acquisition_status(control, status); // Set the acquisition status in the control object

    pthread_mutex_unlock(&lock); // Unlock the mutex after starting acquisition
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to start acquisition: %d\n", result);
    }
    else
    {
        printf("Acquisition started successfully in wallclock mode.\n");
        fflush(stdout); // Flush stdout to ensure immediate output

        nTriggeredSpectra++;
        return true; // Successfully started acquisition
    }

    return false; // Failed to start acquisition
}

static gboolean db_startAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer)
{

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not starting acquisition.\n");

        control_complete_start_acquisition(control, invocation, 0); // Complete the D-Bus method invocation with failure
        return TRUE;                                                // Do not start acquisition if Andor SDK is not active
    }

    float realIntegrationTime, kineticCycleTime, readTime;
    hodr_getAcquisitionTimings(&realIntegrationTime, &kineticCycleTime, &readTime); // Get current acquisition timings
    char acquisitionModeString[64];                                                 // Buffer for acquisition mode string
    hodr_getAcquisitionModeString(acquisitionModeString, 64);                       // Get acquisition mode string
    unsigned int acquisitionMode = hodr_getAcquisitionMode();                       // Get the current acquisition mode from HODR

    char readModeString[64];                    // Buffer for read mode string
    hodr_getReadModeString(readModeString, 64); // Get read mode string

    int seriesLength = hodr_getNumberAcquisitions(); // Get the number of acquisitions captured

    int gainIndex;
    char gainModeString[64];                               // Buffer for gain mode string
    hodr_getCurrentPreAmpGain(&gainIndex, gainModeString); // Get the current preamp gain index
    float gainValue = getPreAmpGainValue(gainIndex);       // Get the current preamp gain value

    printf("Starting acquisition with the following parameters:\n");
    printf("\tIntegration Time: %.5f seconds\n", realIntegrationTime);
    printf("\tAcquisition Mode: %s\n", acquisitionModeString);
    if (acquisitionMode == AC_ACQMODE_KINETIC || acquisitionMode == ACQ_MODE_RUN_TILL_ABORT)
    {
        printf("\tInterval Time: %.5f seconds\n", kineticCycleTime);
    }
    if (acquisitionMode == AC_ACQMODE_KINETIC)
    {
        printf("\tNumber of Captures: %d\n", seriesLength);
    }
    printf("\tRead Mode: %s\n", readModeString);
    printf("\tPreAmp Gain: %.1fx (Index: %d)\n", gainValue, gainIndex);
    printf("\tRead Time: %.5f seconds\n\n", readTime);

    printf("Current Temperature: %.2f, Target Temperature: %.2f\n", control_get_temperature(control), control_get_target_temperature(control));

    printf("Waiting to acquire lock for starting acquisition...\n");
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    printf("Acquired lock for starting acquisition.\n");

    printf("Starting acquisition...\n");
    fflush(stdout); // Flush stdout to ensure immediate output

    unsigned int result = hodr_startAcquisition(); // Start acquisition in HODR

    int status;
    hodr_getStatus(&status); // Get the current acquisition status from HODR

    control_set_acquisition_status(control, status); // Set the acquisition status in the control object

    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to start acquisition: %d\n", result);
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_FAILED, "Failed to start acquisition: %d", result);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        printf("Unlocked mutex after failed acquisition start.\n");
        fflush(stdout);
        return FALSE; // Failed to start acquisition
    }

    printf("Acquisition started successfully.\n");
    fflush(stdout); // Flush stdout to ensure immediate output

    //  generate a new spectrum ID
    uint32_t spectrumID = nTriggeredSpectra++; // Increment the captured spectra count to generate a new spectrum ID

    pthread_mutex_unlock(&lock); // Unlock the mutex after starting acquisition
    printf("Unlocked mutex after starting acquisition.\n");
    printf("New spectrum ID generated: %u\n", spectrumID);

    control_complete_start_acquisition(control, invocation, spectrumID); // Complete the D-Bus method invocation

    return TRUE; // Successfully started acquisition
}

static gboolean db_stopAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer)
{
    printf("Stopping acquisition...\n");
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    printf("Acquired lock for stopping acquisition.\n");
    unsigned int result = hodr_abortAcquisition(); // Abort acquisition in HODR

    int status;
    hodr_getStatus(&status);                         // Get the current acquisition status from HOD
    control_set_acquisition_status(control, status); // Set the acquisition status in the control object
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to abort acquisition: %d\n", result);
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_FAILED, "Failed to abort acquisition: %d", result);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return FALSE;                // Failed to abort acquisition
    }

    printf("Acquisition aborted successfully.\n");
    control_complete_stop_acquisition(control, invocation); // Complete the D-Bus method invocation
    pthread_mutex_unlock(&lock);                            // Unlock the mutex after aborting acquisition
    printf("Unlocked mutex after stopping acquisition.\n");
    return TRUE; // Successfully stopped acquisition
}

static int loadLastSpectrumFromFile(const char *outFile, HODR_AcquisitionParams_t *acquisitionParams, int *data, size_t *dataSize)
{
    // This function can be used to load the last spectrum from the data file if needed.
    // It can be called when the application starts or when a new spectrum is captured.
    // Implement the logic to read the last line from the data file and update the control object accordingly.
    // Get last line from the data file
    printf("Loading last spectrum from data file: %s\n", outFile);
    printf("Waiting to acquire lock for reading data file...\n");
    fflush(stdout);
    pthread_mutex_lock(&dataFileLock); // Lock the mutex for data file operations
    printf("Acquired lock for reading data file.\n");
    fflush(stdout);

    FILE *file = fopen(outFile, "r");
    if (file == NULL)
    {
        pthread_mutex_unlock(&dataFileLock); // Unlock the mutex before returning
        printf("Failed to open data file: %s\n", outFile);
        printf("Unlocked mutex after failed file open.\n");
        fflush(stdout);
        return -1; // Error opening data file
    }

    char line[8192]; // Buffer for reading lines from the file
    // char *line;
    // char *nextLine;
    char nextLine[8192]; // Buffer for the next line
    int nLines = 0;      // Counter for the number of lines read
    while (fgets(line, sizeof(line), file) != NULL)
    {
        nLines++; // Increment the line counter
        // printf("Read line %d: %s", nLines, line); // Log the line read
        fflush(stdout); // Flush stdout to ensure immediate output
        // Store the current line in nextLine
        strncpy(nextLine, line, sizeof(nextLine) - 1);
        nextLine[sizeof(nextLine) - 1] = '\0'; // Ensure null termination
    }

    fclose(file);                        // Close the file after reading
    pthread_mutex_unlock(&dataFileLock); // Unlock the mutex after reading the data file
    printf("Unlocked mutex after reading data file.\n");
    printf("Read %d lines from the data file.\n", nLines); // Log the number of lines read
    fflush(stdout);

    char *linePtr = strdup(line); // Duplicate the last line for parsing
    // second token is timestamp, convert from string to unix timestamp
    printf("Parsing timestamp...\n");
    fflush(stdout);
    // First token is spectrum ID, ignore it
    char token[64];

    char *timeToken = strsep(&linePtr, ",");

    strcpy(token, timeToken); // Copy the token to a buffer

    double exposureTimeDouble;

    // third token is exposure time as float.

    // convert from string to float then multiply by 1e9 to convert to nanoseconds
    char *exposureTimetoken = strsep(&linePtr, ",");
    strcpy(token, exposureTimetoken); // Copy the token to a buffer

    exposureTimeDouble = atof(token);

    // fourth token is preamp gain as float
    double preAmpGainDouble;
    char *preAmpGainToken = strsep(&linePtr, ",");
    printf("Preamp Gain token: %s\n", preAmpGainToken); // Log the preamp gain token
    strcpy(token, preAmpGainToken);                     // Copy the token to a buffer
    preAmpGainDouble = atof(token);
    printf("Preamp Gain parsed: %.2f\n", preAmpGainDouble); // Log the parsed preamp gain

    double temperatureDouble;

    // fifth token is temperature as float
    char *temperatureToken = strsep(&linePtr, ",");

    printf("Temperature token: %s\n", temperatureToken); // Log the temperature token
    strcpy(token, temperatureToken);                     // Copy the token to a buffer

    temperatureDouble = atof(token);
    printf("Temperature parsed: %.2f degrees Celsius\n", temperatureDouble); // Log the parsed temperature

    // sixth token is readMode as int
    int readMode;
    char *readModeToken = strsep(&linePtr, ",");
    printf("Read Mode token: %s\n", readModeToken); // Log the read mode token
    strcpy(token, readModeToken);                   // Copy the token to a buffer

    readMode = atoi(token);
    printf("Read Mode parsed: %d\n", readMode); // Log the parsed read mode

    // seventh token is single track centre as int. This may be an empty string of length 0 if not set. If empty, set to -1
    int singleTrackCentre;
    char *singleTrackCentreToken = strsep(&linePtr, ",");
    printf("Single Track Centre token: %s\n", singleTrackCentreToken); // Log the single track centre token
    strcpy(token, singleTrackCentreToken);                             // Copy the token

    if (strlen(token) == 0)
    {
        singleTrackCentre = -1; // Set to -1 if empty
    }
    else
    {
        singleTrackCentre = atoi(token); // Convert to int if not empty
    }
    printf("Single Track Centre parsed: %d\n", singleTrackCentre); // Log the parsed single track centre

    // eighth token is single track height as int. This may be an empty string of length 0 if not set. If empty, set to -1
    int singleTrackHeight;
    char *singleTrackHeightToken = strsep(&linePtr, ",");
    printf("Single Track Height token: %s\n", singleTrackHeightToken); // Log the single track height token
    strcpy(token, singleTrackHeightToken);                             // Copy the token
    if (strlen(token) == 0)
    {
        singleTrackHeight = -1; // Set to -1 if empty
    }
    else
    {
        singleTrackHeight = atoi(token); // Convert to int if not empty
    }
    printf("Single Track Height parsed: %d\n", singleTrackHeight); // Log the

    printf("Initializing GVariant builder...\n");

    printf("Parsing data values...\n");
    // remaining tokens are data values, split by comma

    // Get the rest of the line and split by comma
    char *dataToken;
    size_t dataCount = 0; // Count of data values
    while ((dataToken = strsep(&linePtr, ",")) != NULL)
    {
        dataCount++;                                  // Increment the data count
        int32_t dataValue = (int32_t)atoi(dataToken); // Convert the token to an integer
        data[dataCount - 1] = dataValue;              // Store the data value in the array
        if (dataCount >= *dataSize)                   // Check if the data count exceeds the allocated size
        {
            fprintf(stderr, "Data count exceeds allocated size: %d\n", (int)*dataSize);
            break; // Exit the loop if the data count exceeds the allocated size
        }
    }

    // parse ISO8691 timestamp from first token, convert to time_t
    struct tm tm;
    // if timestamp includes + or - timezone offset, use format "%Y-%m-%dT%H:%M:%S%z", otherwise use "%Y-%m-%dT%H:%M:%S"

    if (strchr(timeToken, '+') != NULL || strchr(timeToken, '-') != NULL)
    {
        strptime(timeToken, "%Y-%m-%dT%H:%M:%S%z", &tm); // Parse the timestamp with timezone offset into a struct tm
    }
    else
    {
        strptime(timeToken, "%Y-%m-%dT%H:%M:%S", &tm); // Parse the timestamp without timezone offset into a struct tm
    }
    // strptime(timeToken, "%Y-%m-%dT%H:%M:%S", &tm); // Parse the timestamp into a struct tm
    time_t timestamp = mktime(&tm); // Convert struct tm to time_t

    char timestampString[64];                                                       // Buffer for formatted timestamp string
    strftime(timestampString, sizeof(timestampString), "%Y-%m-%dT%H:%M:%S%z", &tm); // Format the timestamp to ISO 8601 format
    printf("Parsed timestamp: %s\n", timestampString);                              // Log the parsed timestamp

    acquisitionParams->timestamp = timestamp;                // Set timestamp in acquisition parameters
    acquisitionParams->integrationTime = exposureTimeDouble; // Set integration time in acquisition parameters
    acquisitionParams->preAmpGain = preAmpGainDouble;        // Set preamp gain in acquisition parameters
    acquisitionParams->temperature = temperatureDouble;      // Set temperature in acquisition parameters
    acquisitionParams->readMode = readMode;                  // Set read mode in acquisition parameters
    acquisitionParams->centre = singleTrackCentre;           // Set single track centre in acquisition parameters
    acquisitionParams->height = singleTrackHeight;           // Set single track height in acquisition parameters

    return 0; // Successfully loaded last spectrum from file
}

static gboolean db_getLastSpectrum(Control *control, GDBusMethodInvocation *invocation, gpointer)
{
    printf("Requesting last captured spectrum data...\n");

    if (nCapturedSpectra == 0)
    {
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_NOT_FOUND, "No spectra captured yet.");

        return FALSE; // No spectra captured yet
    }

    if (!mostRecentDataValid) // Check if the most recent data is valid
    {
        size_t data_size = (size_t)xpixels; // Set the data size to the number of pixels
        if (loadLastSpectrumFromFile(outFile, &mostRecentAcquisitionParams, mostRecentData, &data_size) != 0)
        {
            g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_FAILED, "Failed to load last spectrum from file.");
            return FALSE; // Failed to load last spectrum from file
        }

        mostRecentDataValid = true; // Set the most recent data as valid after loading from file
    }

    GVariantBuilder *builder;
    builder = g_variant_builder_new(G_VARIANT_TYPE("ai")); // Create a new GVariant builder for an array of integers

    // add the data values from the most recent acquisition parameters to the GVariant builder
    for (int i = 0; i < xpixels; i++)
    {
        g_variant_builder_add(builder, "i", mostRecentData[i]); // Add the integer to the GVariant builder
    }

    char timeString[64]; // Buffer for formatted time string

    struct tm *time_info;
    time_info = localtime(&mostRecentAcquisitionParams.timestamp); // Convert the timestamp to UTC time structure

    strftime(timeString, sizeof(timeString), "%Y-%m-%dT%H:%M:%S%z", time_info); // Format the timestamp to ISO 8601 format

    GVariant *response;
    response = g_variant_new("(sdddiiiai)",
                             timeString,
                             (double)mostRecentAcquisitionParams.integrationTime,
                             (double)mostRecentAcquisitionParams.preAmpGain,
                             (double)mostRecentAcquisitionParams.temperature,
                             mostRecentAcquisitionParams.readMode,
                             mostRecentAcquisitionParams.centre,
                             mostRecentAcquisitionParams.height,
                             builder); // Create a response GVariant with timestamp, exposure time, temperature etc., and data array

    control_complete_get_data(control, invocation, response); // Complete the D-Bus method invocation with the GVariant

    fflush(stdout); // Flush stdout to ensure immediate output
    return TRUE;    // Successfully returned the last spectrum data
}

static gboolean db_setWallclockAcquisitionInterval(Control *control, GDBusMethodInvocation *invocation, unsigned int interval, gpointer)
{

    printf("Setting wallclock acquisition interval to %d minutes...\n", interval);
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    if (interval <= 0)
    {
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT, "Invalid wallclock acquisition interval: %.5f", (float)interval);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return TRUE;                 // Invalid wallclock acquisition interval
    }
    // guint wallclockInterval = interval; // Set the wallclock acquisition interval

    control_set_wallclock_interval(control, interval); // Set the wallclock acquisition interval property in the control object

    control_complete_set_wallclock_interval(control, invocation, TRUE); // Complete the D-Bus method invocation with success

    if (timerHandle != 0)
    {
        g_source_remove(timerHandle);              // Remove the existing timer source if it exists
        timerHandle = 0;                           // Reset the timer handle
        setNextAcquisition(&timerHandle, control); // Set the next acquisition time based on the new wallclock acquisition interval
    }
    printf("Wallclock acquisition interval set to %d minutes successfully.\n", interval);
    pthread_mutex_unlock(&lock); // Unlock the mutex after setting the wallclock acquisition interval
    return TRUE;                 // Successfully set wallclock acquisition interval
}

static gboolean db_startWallclockAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer)
{

    printf("Starting wallclock acquisition...\n");
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not starting wallclock acquisition.\n");
        control_complete_start_wallclock_acquisition(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        pthread_mutex_unlock(&lock);                                              // Unlock the mutex before returning
        return TRUE;                                                              // Do not start wallclock acquisition if Andor SDK is not active
    }

    control_set_wallclock_acquisition_active(control, TRUE); // Set the wallclock acquisition active property in the control object
    setNextAcquisition(&timerHandle, control);               // Set the next acquisition time based on the wallclock acquisition interval

    control_complete_start_wallclock_acquisition(control, invocation, TRUE); // Complete the D-Bus method invocation with success
    printf("Wallclock acquisition started successfully.\n");
    pthread_mutex_unlock(&lock); // Unlock the mutex after starting wallclock acquisition
    return TRUE;                 // Successfully started wallclock acquisition
}

static gboolean db_stopWallclockAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer)
{

    printf("Stopping wallclock acquisition...\n");
    fflush(stdout);            // Flush stdout to ensure immediate output
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not stopping wallclock acquisition.\n");
        fflush(stdout);                                                          // Flush stdout to ensure immediate output
        control_complete_stop_wallclock_acquisition(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        pthread_mutex_unlock(&lock);                                             // Unlock the mutex before returning
        return TRUE;                                                             // Do not stop wallclock acquisition if Andor SDK is not active
    }
    control_set_wallclock_acquisition_active(control, FALSE); // Set the wallclock acquisition active property in the control object
    printf("Removing timer source with ID: %u\n", timerHandle);
    g_source_remove(timerHandle); // Remove the timer source to stop wallclock acquisition
    timerHandle = 0;              // Reset the timer handle

    control_set_wallclock_next_capture(control, 0); // Reset the next capture time

    control_complete_stop_wallclock_acquisition(control, invocation, TRUE); // Complete the D-Bus method invocation with success
    printf("Wallclock acquisition stopped successfully.\n");
    fflush(stdout);              // Flush stdout to ensure immediate output
    pthread_mutex_unlock(&lock); // Unlock the mutex after stopping wallclock acquisition
    return TRUE;                 // Successfully stopped wallclock acquisition
}

int createDataFile(char *directory, char *filename)
{

    time_t current_time;
    struct tm *time_info;
    char timeString[64]; // space for "YYYY-MM-DD_HH-MM-SS_andor.csv\0"

    time(&current_time);
    time_info = localtime(&current_time);

    strftime(timeString, sizeof(timeString), "%Y-%m-%d_andor.csv", time_info);

    // // fill filename with zeroes
    // memset(filename, 0, sizeof(filename));

    sprintf(filename, "%s/%s", directory, timeString);

    int filepathLength = strlen(filename);
    FILE *file = fopen(filename, "a");
    if (file == NULL)
    {
        fprintf(stderr, "Error creating file %s: %s\n", filename, strerror(errno));
        return -1; // Error creating file
    }
    fclose(file);
    return filepathLength; // Return the length of the file path
}

int countLines()
{
    pthread_mutex_lock(&dataFileLock); // Lock the mutex for data file operations#
    FILE *file = fopen(outFile, "r");
    size_t buffSize = 65536;
    char buf[buffSize];
    int counter = 0;
    for (;;)
    {
        size_t res = fread(buf, 1, buffSize, file);
        if (ferror(file))
        {
            pthread_mutex_unlock(&dataFileLock); // unLock the mutex for data file operations
            return -1;
        }

        size_t i;
        for (i = 0; i < res; i++)
        {
            if (buf[i] == '\n')
            {
                counter++;
            }
        }

        if (feof(file))
        {
            break;
        }
    }
    pthread_mutex_unlock(&dataFileLock); // unLock the mutex for data file operations
    return counter;
}

int appendToFile(float *integrationTime, float *preAmpGain, double *temperature, int *readMode, int *singleTrackHeight, int *singleTrackCentre, const int32_t *data, size_t size)
{

    // copy the data to mostRecentData array for later retrieval
    memcpy(mostRecentData, data, size * sizeof(int32_t));
    mostRecentDataValid = true; // Mark the most recent data as valid
    mostRecentAcquisitionParams.integrationTime = *integrationTime;
    mostRecentAcquisitionParams.preAmpGain = *preAmpGain;
    mostRecentAcquisitionParams.temperature = *temperature;
    mostRecentAcquisitionParams.readMode = *readMode;
    mostRecentAcquisitionParams.height = *singleTrackHeight;
    mostRecentAcquisitionParams.centre = *singleTrackCentre;

    createDataFile(dataDir, outFile);
    pthread_mutex_lock(&dataFileLock); // Lock the mutex to ensure thread safety for file operations
    FILE *file = fopen(outFile, "a");
    if (file == NULL)
    {
        fprintf(stderr, "Error opening file %s for appending.\n", outFile);
        return -1;
    }

    printf("Appending data to file %s...\n", outFile);
    time_t current_time;
    struct tm *time_info;
    char timeString[64]; // space for "HH:MM:SS\0"

    time(&current_time);
    // subtract the exposure time from the current time to get the timestamp
    current_time -= (time_t)(*integrationTime);
    time_info = localtime(&current_time);

    mostRecentAcquisitionParams.timestamp = current_time; // Store the timestamp in the most recent acquisition parameters

    strftime(timeString, sizeof(timeString), "%Y-%m-%dT%H:%M:%S%z", time_info);

    printf("Writing to file:\n");

    float gainValue = getPreAmpGainValue(mostRecentAcquisitionParams.preAmpGain);

    fprintf(file, "%s,", timeString);                                    // Write timestamp
    fprintf(file, "%.5f,", mostRecentAcquisitionParams.integrationTime); // Write exposure time
    fprintf(file, "%.1f,", gainValue);                                   // Write pre-amplifier gain
    fprintf(file, "%.2f,", mostRecentAcquisitionParams.temperature);     // Write temperature
    fprintf(file, "%d,", mostRecentAcquisitionParams.readMode);          // Write read mode
    if (*readMode == READ_MODE_SINGLE_TRACK)
    {
        fprintf(file, "%d,", mostRecentAcquisitionParams.height); // Write single track height if in single track mode
        fprintf(file, "%d,", mostRecentAcquisitionParams.centre); // Write single track centre if in single track mode
    }
    else
    {
        fprintf(file, ",,"); // Write empty values for single track height and centre if not in single track mode
    }
    for (size_t i = 0; i < size; i++)
    {
        fprintf(file, "%d", mostRecentData[i]);
        if (i < size - 1)
        {
            fprintf(file, ",");
        }
    }
    fprintf(file, "\n"); // New line after each data set

    printf("Data appended successfully.\n");
    fflush(file); // Ensure data is written to file
    fclose(file);
    nCapturedSpectra++;                  // Increment the captured spectra count
    pthread_mutex_unlock(&dataFileLock); // Unlock the mutex after file operations

    printf("Unlocked mutex after appending data to file.\n");
    return 0;
}

int32_t findMaxIntensity(int32_t *data, size_t size, size_t startIndex, size_t endIndex)
{
    if (size == 0 || startIndex >= size || endIndex > size || startIndex >= endIndex)
    {
        fprintf(stderr, "Invalid parameters for findMaxIntensity: size=%zu, startIndex=%zu, endIndex=%zu\n", size, startIndex, endIndex);
    }
    int32_t maxIntensity = data[startIndex]; // Initialize maxIntensity with the first element of the data array
    for (size_t i = 1; i < endIndex; i++)
    {
        if (data[i] > maxIntensity)
        {
            maxIntensity = data[i]; // Update maxIntensity if a larger value is found
        }
    }
    return maxIntensity; // Return the maximum intensity found in the data array
}

int captureDuringAuto(Control *control, int32_t *data, size_t xpixels, float *integrationTime)
{

    float realIntegrationTime, kineticCycleTime, readoutTime;
    hodr_getAcquisitionTimings(&realIntegrationTime, &kineticCycleTime, &readoutTime); // Get the actual acquisition timings from HODR

    *integrationTime = realIntegrationTime; // Update the integration time with the actual value
    int readMode = hodr_getReadMode();      // Get the current read mode from HODR

    char readModeString[32];                    // Buffer for read mode string
    hodr_getReadModeString(readModeString, 32); // Get the read mode

    int preAmpGainIndex;
    char gainString[32];                                     // Buffer for pre-amplifier gain string
    hodr_getCurrentPreAmpGain(&preAmpGainIndex, gainString); // Get the current pre-amplifier gain index from HODR

    float preAmpGain = getPreAmpGainValue(preAmpGainIndex); // Get the pre-amplifier gain value based on the index

    int centre, height;
    hodr_getSingleTrack(&centre, &height); // Get the current Single Track settings

    int result = hodr_startAcquisition(); // Start acquisition in HODR

    int status;
    hodr_getStatus(&status);                         // Get the current acquisition status from HODR
    control_set_acquisition_status(control, status); // Set the acquisition status in the control object

    printf("Starting acquisition with params:\n");
    printf("\tIntegration Time: %.5f seconds\n\tPre-Amp Gain: %.1f\n\tRead Mode: %s\n\tSingle Track Centre: %d\n\tSingle Track Height: %d\n", realIntegrationTime, preAmpGain, readModeString, centre, height);

    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to start acquisition: %d\n", result);
        return -1; // Error starting acquisition
    }

    result = WaitForAcquisition(); // Wait for acquisition to finish

    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Error waiting for acquisition: %d\n", result);
        return -1; // Error waiting for acquisition
    }

    hodr_getStatus(&status);                         // Get the current acquisition status from HODR
    control_set_acquisition_status(control, status); // Set the acquisition status in the control object

    result = hodr_getMostRecentImage(data, xpixels); // Get the most recent image acquired

    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Error getting images: %d\n", result);
        return -1; // Error getting images
    }

    nCapturedSpectra++; // Increment the captured spectra count

    result = appendToFile(&realIntegrationTime, &preAmpGain, &lastTemperature, &readMode, &centre, &height, data, xpixels); // Append data to output file

    control_emit_acquisition_finished(control, nCapturedSpectra); // Emit acquisition finished signal

    return 0; // Successfully captured data
}

void adjustIntegrationTime(Control *control, unsigned int targetIntensity, float integrationTime, int32_t *data, size_t size, unsigned int attemptLimit)
{

    int32_t maxIntensity; // Variable to hold the maximum intensity found in the data

    // //float previousIntegrationTime = 0; // Store the previous integration time for comparison
    float newIntegrationTime; // Variable to hold the new integration time calculated based on the ratio

    unsigned int attemptCount = 0; // Counter for the number of adjustment attempts made

    while (true)
    {

        if (attemptCount >= attemptLimit) // Check if the maximum number of attempts has been reached
        {
            printf("Maximum adjustment attempts (%d) reached. Stopping adjustments.\n", attemptLimit);
            fflush(stdout);
            break; // Exit the loop if the maximum number of attempts has been reached
        }
        attemptCount++; // Increment the attempt counter

        printf("********************************************************************************\n");
        printf("Attempt %d: Adjusting integration time...\n", attemptCount);
        fflush(stdout);

        int readMode = hodr_getReadMode();        // Get the current read mode from HODR
        control_set_read_mode(control, readMode); // Set the read mode in the control object

        char readModeString[32];                    // Buffer for read mode string
        hodr_getReadModeString(readModeString, 32); // Get the read mode string from HODR
        printf("Current read mode: %s\n", readModeString);
        fflush(stdout);
        maxIntensity = findMaxIntensity(data, size, 5, size - 5); // Find the maximum intensity in the data

        float ratio = (float)maxIntensity / (float)targetIntensity; // Calculate the ratio of max intensity to target intensity

        if (ratio >= 0.95 && ratio <= 1.05) // Check if the ratio is within the acceptable range (5% tolerance)
        {
            printf("Integration time is optimal. No adjustment needed.\n");
            fflush(stdout);
            break; // Exit the loop if the integration time is optimal
        }

        printf("Integration time: %.5f seconds\n", integrationTime);
        printf("Max intensity: %d, Target intensity: %d, Ratio: %.2f\n", maxIntensity, targetIntensity, ratio);
        fflush(stdout);

        newIntegrationTime = integrationTime / ratio; // Calculate the new integration time based on the ratio

        if (maxIntensity >= 65535) // Check if max intensity is at or above the saturation threshold
        {
            newIntegrationTime = integrationTime * 0.25; // Reduce integration time by 75% if saturated
            printf("SATURATED - Reducing integration time to %.5f seconds.\n", newIntegrationTime);
        }

        if (readMode == READ_MODE_FVB && newIntegrationTime <= 0.001) // Check if integration time is already at or below the minimum threshold
        {
            hodr_setReadMode(READ_MODE_SINGLE_TRACK);               // Set read mode to single track if saturated and integration time is too low
            control_set_read_mode(control, READ_MODE_SINGLE_TRACK); // Set acquisition mode in the control object to single track
            hodr_setPreAmpGain(0);                                  // Set preamp gain to 0 (1x)
            control_set_pre_amp_gain(control, 0);                   // Set preamp gain in the control object
            float oldIntegrationTime = integrationTime;             // Store the old integration time for logging
            newIntegrationTime = integrationTime * 2;
            printf("SATURATED - Integration time of %.5f is too low. Switching to Single Track mode. New integration time: %.5f seconds.\n", oldIntegrationTime, newIntegrationTime);
            fflush(stdout);
        }

        if (readMode == READ_MODE_SINGLE_TRACK && newIntegrationTime > 0.5) // Check if integration time is above the maximum threshold for single track mode
        {
            hodr_setReadMode(READ_MODE_FVB);               // Set read mode to FVB if integration time is too high
            control_set_read_mode(control, READ_MODE_FVB); // Set acquisition mode in the control object to FVB
            hodr_setPreAmpGain(1);                         // Set preamp gain to 1 (1.7x)
            control_set_pre_amp_gain(control, 1);          // Set preamp gain in the control object
            float oldIntegrationTime = integrationTime;    // Store the old integration time for logging
            newIntegrationTime = integrationTime * 0.5;
            printf("Integration time of %.5f is too high for Single Track mode. Switching to FVB mode. New integration time: %.5f seconds.\n", oldIntegrationTime, newIntegrationTime);
            fflush(stdout);
        }

        if (newIntegrationTime >= maximumIntegrationTime) // Check if the new integration time exceeds the maximum allowed integration time
        {
            newIntegrationTime = maximumIntegrationTime; // Limit the new integration time to the maximum allowed value
            printf("New integration time exceeds maximum allowed. Limiting to %.1f seconds.\n", newIntegrationTime);
            fflush(stdout);
        }

        hodr_setExposureTime(newIntegrationTime); // Set the new integration time in HODR

        float realIntegrationTime, kineticCycleTime, readoutTime;
        hodr_getAcquisitionTimings(&realIntegrationTime, &kineticCycleTime, &readoutTime); // Get the actual acquisition timings from HODR

        printf("New integration time: %.5f seconds\n", realIntegrationTime);

        control_set_integration_time_secs(control, realIntegrationTime); // Set the new integration time in the control object
        control_set_interval_time_secs(control, kineticCycleTime);       // Set the new kinetic cycle time in the control object

        // If in single track mode, also capture an FVB spectrum at low integration time
        if (readMode == READ_MODE_SINGLE_TRACK)
        {
            printf("Capturing FVB spectrum at low integration time for comparison...\n");
            fflush(stdout);
            hodr_setReadMode(READ_MODE_FVB); // Set read mode to FVB

            hodr_setPreAmpGain(1); // Set preamp gain to 1 (1.7x)

            hodr_setExposureTime(realIntegrationTime * 0.01); // Set integration time to 1% of the current integration time for FVB capture

            captureDuringAuto(control, data, size, &integrationTime); // Capture data with the new integration time

            hodr_setReadMode(READ_MODE_SINGLE_TRACK); // Set read mode back to Single Track
            hodr_setPreAmpGain(0);                    // Set preamp gain back to 0 (1x)
            hodr_setExposureTime(newIntegrationTime); // Set integration
        }

        fflush(stdout);

        int result = captureDuringAuto(control, data, size, &integrationTime); // Capture data with the new integration time
        if (result != 0)
        {
            fprintf(stderr, "Error capturing data during auto adjustment: %d\n", result);
            return; // Error capturing data
        }
    }
}

void setNextAcquisition(int *timeoutID, Control *control) // Function to set the next acquisition
{

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Cannot set next acquisition.\n");
        fflush(stdout); // Flush stdout to ensure immediate output
        return;         // Do not proceed if Andor SDK is not active
    }

    guint wallclockInterval = control_get_wallclock_interval(control);            // Get the wallclock interval from the control object
    gboolean wallclockActive = control_get_wallclock_acquisition_active(control); // Get the wallclock acquisition active status from the control object

    printf("Current wallclock interval: %d minutes\n", wallclockInterval);
    fflush(stdout); // Flush stdout to ensure immediate output
    if (wallclockInterval == 0 || !wallclockActive)
    {
        printf("Acquisition schedule interval is not set.Not setting next acquisition.\n");
        fflush(stdout); // Flush stdout to ensure immediate output
        return;         // Do not proceed if acquisition schedule interval is not set
    }

    // acquire lock to ensure thread safety
    printf("Setting next acquisition...\n");
    printf("Waiting to acquire lock for setting next acquisition...\n");
    fflush(stdout);

    printf("Acquired lock for setting next acquisition.\n");
    fflush(stdout);
    unsigned int seconds = secondsToNextHourFraction(wallclockInterval); // Calculate seconds to next hour fraction

    time_t currentTime = time(NULL);              // Get the current time
    struct tm *tm_info = localtime(&currentTime); // Convert to local time
    printf("Current time: %s", asctime(tm_info)); // Log the current time
    fflush(stdout);
    time_t nextAcquisitionTime = currentTime + seconds;    // Calculate the next acquisition time
    tm_info = localtime(&nextAcquisitionTime);             // Convert to local time
    printf("Next acquisition time: %s", asctime(tm_info)); // Log the next acquisition time
    fflush(stdout);

    control_set_wallclock_next_capture(control, (guint)nextAcquisitionTime); // Set the next acquisition time in the control object

    if (seconds == 0)
    {
        printf("Next acquisition is scheduled for the current time. Starting acquisition immediately.\n");
        fflush(stdout);                               // Flush stdout to ensure immediate output
        StartAcquisitionIfPossibleWallclock(control); // Start acquisition immediately if seconds is 0
        timeoutID = 0;                                // Reset the timeout ID since acquisition starts immediately

        return; // Do not set a timeout if acquisition starts immediately
    }

    printf("Setting next acquisition in %u seconds...\n", seconds);
    fflush(stdout);

    if (*timeoutID != 0)
    {
        printf("Removing existing timeout with ID: %u\n", *timeoutID);
        g_source_remove(*timeoutID); // Remove the existing timeout if it exists
        *timeoutID = 0;              // Reset the timeout ID
    }

    *timeoutID = g_timeout_add_seconds(seconds, (GSourceFunc)StartAcquisitionIfPossibleWallclock, control); // Set a timeout to handle the acquisition loop

    printf("Setting next acquisition with timeout ID: %u\n", *timeoutID);

    printf("Next acquisition set successfully.\n");
    fflush(stdout);
}

void *handleAcquisitionLoop(void *arg) // Function to handle the acquisition loop
{
    Control *control = (Control *)arg;

    if (!andorActive) // Check if Andor SDK is active
    {
        fprintf(stderr, "Andor SDK is not active. Cannot handle acquisition loop.\n");
        return NULL; // Do not proceed if Andor SDK is not active
    }
    unsigned int result;
    unsigned int acquisitionStatus = 0; // Variable to hold acquisition status
    printf("Handling acquisition loop...\n");

    while (true)
    {
        if (!andorActive) // Check if Andor SDK is still active
        {
            printf("Andor SDK is not active. Exiting acquisition loop.\n");
            break; // Exit the loop if Andor SDK is not active
        }
        printf("Waiting for next acquisition to finish...\n");
        fflush(stdout);                           // Flush stdout to ensure immediate output
        acquisitionStatus = WaitForAcquisition(); // Start waiting for acquisition data

        int status;
        hodr_getStatus(&status);                         // Get the current acquisition status from HODR
        control_set_acquisition_status(control, status); // Set the acquisition status in the control object

        if (!andorActive) // Check if Andor SDK is still active after waiting
        {
            printf("Andor SDK is not active. Exiting acquisition loop.\n");
            break; // Exit the loop if Andor SDK is not active
        }
        if (acquisitionStatus != DRV_SUCCESS)
        {
            fprintf(stderr, "Acquisition Wait cancelled.\n");
            return NULL; // Error waiting for acquisition
        }

        printf("Acquisition finished - spectrum #%d\n", nCapturedSpectra + 1);
        fflush(stdout); // Flush stdout to ensure immediate output
        printf("Waiting to acquire lock for processing acquired data...\n");
        fflush(stdout);
        pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
        printf("Acquired lock for processing acquired data.\n");
        fflush(stdout);

        int32_t data[xpixels]; // Array to hold the acquired data

        result = hodr_getMostRecentImage(data, (size_t)xpixels); // Get the most recent image acquired

        if (result != DRV_SUCCESS)
        {
            fprintf(stderr, "Error getting images: %d\n", result);
            pthread_mutex_unlock(&lock); // Unlock the mutex before returning
            continue;                    // Skip to the next iteration if there was an error
        }

        control_emit_acquisition_finished(control, nCapturedSpectra + 1); // Emit signal that acquisition has finished

        printf("Acq. %d: Acquired data for spectrum. Result: %d\n", nCapturedSpectra + 1, result);
        fflush(stdout);
        float integrationTime, kineticCycleTime, readoutTime;
        hodr_getAcquisitionTimings(&integrationTime, &kineticCycleTime, &readoutTime); // Get acquisition timings
        printf("Acq. %d: Acquisition timings - Exposure: %.6f, Kinetic Cycle: %.6f, Readout: %.6f\n", nCapturedSpectra + 1, integrationTime, kineticCycleTime, readoutTime);

        int preAmpGainIndex;
        char preAmpGainString[32]; // Buffer for pre-amplifier gain string

        hodr_getCurrentPreAmpGain(&preAmpGainIndex, preAmpGainString); // Get the pre-amplifier gain index
        float preAmpGain = getPreAmpGainValue(preAmpGainIndex);        // Get the pre-amplifier gain value based on the index

        int readMode = hodr_getReadMode(); // Get the current read mode from HODR
        int centre, height;
        hodr_getSingleTrack(&centre, &height); // Get the current Single Track settings

        fflush(stdout);
        printf("Acq. %d: Last temperature: %.2f\n", nCapturedSpectra + 1, lastTemperature); // Log the last temperature
        fflush(stdout);
        result = appendToFile(&integrationTime, &preAmpGain, &lastTemperature, &readMode, &centre, &height, data, xpixels); // Append data to output file
        control_set_number_spectra(control, nCapturedSpectra);
        unsigned int targetIntensity = control_get_target_intensity(control); // Get the target intensity from the control object
        printf("Target intensity: %d\n", targetIntensity);                    // Log the target intensity
        fflush(stdout);

        if (targetIntensity > 0)
        {
            printf("Acq. %d: Target intensity: %d at integration time %.5fs\n", nCapturedSpectra + 1, targetIntensity, integrationTime); // Log the target intensity
            fflush(stdout);
            adjustIntegrationTime(control, targetIntensity, integrationTime, data, xpixels, adjustmentAttemptLimit); // Adjust integration time based on target intensity
        }
        control_set_number_spectra(control, nCapturedSpectra);
        // hodr_getAcquisitionTimings(&integrationTime, &kineticCycleTime, &readoutTime);     // Get acquisition timings

        control_set_data_path(control, outFile); // Set the data path in the control object

        setNextAcquisition(&timerHandle, control); // Set the next acquisition time based on the wallclock acquisition interval
        printf("Data appended to file. Result: %d, N captured spectra: %d\n", result, nCapturedSpectra);
        fflush(stdout);
        pthread_mutex_unlock(&lock); // Unlock the mutex after processing
        printf("Acq. %d: Unlocked mutex after processing acquired data.\n", nCapturedSpectra);
        fflush(stdout);
    }

    return NULL; // Return NULL to indicate the thread has finished
}
