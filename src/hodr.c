#include "atmcdLXd.h"
#include "hodr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdbool.h>
#include <sys/select.h>
#include <pthread.h>
#include <semaphore.h>
#include <gio/gio.h>
#include <signal.h>
#include "control.h"

#define SHUTTER_TYP_OPEN_LOW 0
#define SHUTTER_TYP_OPEN_HIGH 1
#define SHUTTER_MODE_FULLY_AUTO 0
#define READ_MODE_FVB 0
#define READ_MODE_MULTI_TRACK 1
#define READ_MODE_RANDOM_TRACK 2
#define READ_MODE_SINGLE_TRACK 3
#define READ_MODE_IMAGE 4



pthread_mutex_t lock;
pthread_mutex_t endThreadLock;
pthread_mutex_t dataFileLock;        // Mutex for data file operations
pthread_mutex_t acquisitionLoopLock; // Mutex for acquisition loop operations
bool endThread = false;              // Flag to signal the command thread to end

uint32_t initialMode = 1; // Initial acquisition mode
uint32_t initialTargetIntensity = 60000; // Target intensity for the acquisition
uint32_t adjustmentAttemptLimit = 10; // Limit for adjustment attempts during acquisition
float initialIntegrationTime = 0.01f; // Initial integration time for the acquisition
float minimumIntegrationTime = 0.00001f; // Minimum integration time for the acquisition
float maximumIntegrationTime = 900.0f; // Maximum integration time for the acquisition

uint32_t initialPreAmpGainIndex = 0; // Initial pre-amplifier gain index (0 for 1x, 1 for 1.7x)
int32_t initialTargetTemperature = -70;      // Initial target temperature for the acquisition
bool initialCoolerMode = true;                     // Initial cooler mode (true for ON, false for OFF)

int32_t initialReadMode = 0; // Initial read mode for the acquisition
int32_t initialSingleTrackHeight = 20; // Initial single track height for the acquisition
int32_t initialSingleTrackCentre = 10; // Initial single track centre for the acquisition

bool initialWallclockActive = true; // Initial wallclock acquisition active status
// Must be a factor of 60 to ensure proper timing repeating every hour.
uint32_t initialWallclockInterval = 2; // Initial wallclock acquisition interval in minutes (0 means no schedule)

//char andorFile[256] = "../miniforge3/pkgs/andor2-sdk-2.104.30064-0/etc/andor/";
char initFile[256] = "./hodr.conf"; // Configuration file for HODR
char andorFile[256] = "../andor/etc/";
char outFile[256]; // Output file for data

int countLines();
int createDataFile(char *directory, char *filename);
int appendToFile(float *exposureTime, float *preAmpGain, double *temperature, int *readMode, int *singleTrackHeight, int *singleTrackCentre, const int32_t *data, size_t size);
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
static gboolean db_startAcquisition(Control *control, GDBusMethodInvocation *invocation, gdouble integration_time, gboolean pre_amp_gain, gdouble interval_time, guint mode, guint number, gpointer user_data);
static gboolean db_setAcquisitionMode(Control *control, GDBusMethodInvocation *invocation, guint32 mode, gpointer user_data);
static gboolean db_stopAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_setInterval(Control *control, GDBusMethodInvocation *invocation, gdouble interval, gpointer user_data);
static gboolean db_getLastSpectrum(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_setTargetIntensity(Control *control, GDBusMethodInvocation *invocation, guint intensity, gpointer user_data);
static gboolean db_startWallclockAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_stopWallclockAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_setWallclockAcquisitionInterval(Control *control, GDBusMethodInvocation *invocation, guint interval, gpointer user_data);


static gboolean StartAcquisitionIfPossible(gpointer controlPtr);
static gboolean StartAcquisitionIfPossibleWallclock(gpointer controlPtr);

void setNextAcquisition(int *timeoutID, Control *control); // Function to set the next acquisition
// static gboolean db_getData(Control *control, GDBusMethodInvocation *invocation, gint ref, gpointer user_data);

void *handleAcquisitionLoop();

char dataDir[256] = "../hodr_data"; // Directory for data files

const float preAmpGainValues[2] = {1.0, 1.7}; // Array of pre-amplifier gain values

double lastTemperature = 0.0;       // Last temperature read from the device
double lastTargetTemperature = 0.0; // Last target temperature set
int lastTemperatureStatus = 0;      // Last temperature status read from the device

uint32_t nTriggeredSpectra = 0; // Number of triggered spectra
uint32_t nCapturedSpectra = 0;  // Number of captured spectra

int xpixels, ypixels; // Detector size
GMainLoop *loop;

guint dataWaitFunctionRef;

//HODR_Config_t *hodr_cfg; // Pointer to HODR configuration structure

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
        AbortAcquisition(); // Abort any ongoing acquisition
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

        if (initialMode == 0) {
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
    g_signal_connect(control, "handle-set_pre_amp_gain", G_CALLBACK(db_setPreAmpGain), NULL);                          // Connect the signal for setting preamp gain
    g_signal_connect(control, "handle-set_read_mode", G_CALLBACK(db_setReadMode), NULL);                              // Connect the signal for setting read mode
    g_signal_connect(control, "handle-set_single_track", G_CALLBACK(db_setSingleTrack), NULL);                        // Connect the signal for setting single track parameters
    g_signal_connect(control, "handle-stop_acquisition", G_CALLBACK(db_stopAcquisition), NULL);                       // Connect the signal for stopping acquisition
    g_signal_connect(control, "handle-set_target_intensity", G_CALLBACK(db_setTargetIntensity), NULL);                // Connect the signal for setting target intensity
    g_signal_connect(control, "handle-set_interval", G_CALLBACK(db_setInterval), NULL);                               // Connect the signal for setting interval
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

    control_set_over_heat_reboot_count(control, 0);             // Initialize overheat reboot count to 0

    gboolean preAmpGainSet = initialPreAmpGainIndex == 1;

    control_set_pre_amp_gain(control, preAmpGainSet); // Set the initial pre-amp gain in the control object

    control_set_temperature(control, initialTargetTemperature); // Set the initial target temperature in the control object

    control_set_target_intensity(control, initialTargetIntensity);                 // Set the target intensity in the control object
    control_set_target_temperature(control, initialTargetTemperature);             // Set the target temperature in the control object
    control_set_wallclock_interval(control, initialWallclockInterval);                    // Set wallclock acquisition interval in the control object
    control_set_wallclock_acquisition_active(control, initialWallclockActive); // Set wallclock acquisition active status in the control object

    activateHodr(control);                                                         // Activate HODR

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

    pthread_mutex_lock(&lock);                                            // Lock the mutex to ensure thread safety
    unsigned int result = hodr_init(andorFile, outFile, false); // Initialize HODR
    if (result == DRV_SUCCESS)
    {
        andorActive = true;                // Set Andor SDK active flag to TRUE
        control_set_active(control, TRUE); // Set the control object as active
        control_set_live(control, TRUE);   // Initialize live status to TRUE



    

        printf("HODR activated successfully.\n");
        hodr_setCoolerMode(true); // Turn on the cooler

        control_set_over_heat_count(control, 0);                           // Set the overheat count in the control object
        control_set_over_heat_cooldown_active(control, FALSE);             // Set cooldown active in the control object
        float targetTemp = control_get_target_temperature(control);        // Get target temperature from control object
        hodr_setTargetTemperature(targetTemp);                             // Set target temperature in HODR
        printf("Target temperature set to: %.2f\n", targetTemp);
        fflush(stdout);                                                     // Flush stdout to ensure immediate output
        g_timeout_add_seconds(1, (GSourceFunc)db_getTemperature, control); // Schedule next temperature check
        hodr_getDetectorSize(&xpixels, &ypixels);                          // Get detector size
        printf("Detector size: %d x %d pixels\n", xpixels, ypixels);

        hodr_setPreAmpGain(control_get_pre_amp_gain(control) ? 1 : 0); // Set pre-amp gain based on control object

        int preAmpGainIndex;
        char gainString[32];
        hodr_getCurrentPreAmpGain(&preAmpGainIndex, gainString);

        float integrationTime, accumulate, kinetic;

        result = GetAcquisitionTimings(&integrationTime, &accumulate, &kinetic);
        if (result != DRV_SUCCESS)
        {
            fprintf(stderr, "Failed to get acquisition timings: %d\n", result);
        } else
        {
            printf("Current Acquisition Timings - Exposure: %.6f s, Accumulate: %.6f s, Kinetic: %.6f s\n", integrationTime, accumulate, kinetic);
            control_set_integration_time_secs(control, integrationTime); // Set the integration time in the control object
        }

        hodr_setReadMode(initialReadMode); // Set read mode to Full Vertical Binning
        hodr_setSingleTrack(initialSingleTrackCentre, initialSingleTrackHeight); // Set single track parameters

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
        hodr_getStatus(&status); // Get the current acquisition status from HOD
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
    CancelWait();                        // Cancel any ongoing wait operations
    AbortAcquisition();                  // Abort any ongoing acquisition

    
    int status;
    hodr_getStatus(&status); // Get the current acquisition status from HOD
    control_set_acquisition_status(control, status); // Set the acquisition status in the control object

    g_source_remove(timerHandle); // Remove the timer for wallclock acquisition
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
            fflush(stdout); // Flush stdout to ensure immediate output
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
    fflush(stdout); // Flush stdout to ensure immediate output
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    printf("Mutex lock acquired for setting integration time.\n");
    fflush(stdout); // Flush stdout to ensure immediate output

    if (int_time < 0)
    {
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT, "Invalid integration time: %.9f", int_time);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return TRUE;                 // Invalid integration time
    }

    control_set_target_intensity(control, 0); // Set target intensity to 0 before changing integration time
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
    fflush(stdout); // Flush stdout to ensure immediate output
    control_complete_set_integration_time(control, invocation, TRUE); // Complete the D-Bus method invocation with success
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

    hodr_setSingleTrack(10, 20); // Set single track parameters in HODR (example values)

    control_set_read_mode(control, mode); // Update read mode in control object
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
    control_complete_set_single_track(control, invocation, TRUE); // Complete the D-Bus method invocation with success
    printf("Single track set from %d to %d successfully.\n", centre, height);
    pthread_mutex_unlock(&lock); // Unlock the mutex after setting the single track
    return TRUE;                 // Successfully set single track
}

static gboolean db_setPreAmpGain(Control *control, GDBusMethodInvocation *invocation, gboolean value, gpointer)
{

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not setting preamp gain.\n");
        control_complete_set_pre_amp_gain(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        return FALSE;                                                 // Do not update if Andor SDK is not active
    }
    printf("Setting preamp gain to index %d...\n", value);
    fflush(stdout); // Flush stdout to ensure immediate output
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    unsigned int result = hodr_setPreAmpGain((int)value); // Set preamp gain in HODR
    if (result != DRV_SUCCESS)
    {
        control_complete_set_pre_amp_gain(control, invocation, FALSE); // Complete the D-Bus method invocation with failure
        printf("Failed to set preamp gain: %d\n", result);
        fflush(stdout);                // Flush stdout to ensure immediate output
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return FALSE;                // Error setting preamp gain
    }
    control_complete_set_pre_amp_gain(control, invocation, TRUE); // Complete the D-Bus method invocation with success
    printf("Preamp gain set to index %d successfully.\n", value);
    fflush(stdout); // Flush stdout to ensure immediate output
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
    hodr_getStatus(&status); // Get the current acquisition status from HOD
    control_set_acquisition_status(control, status); // Set the acquisition status in the control object

    return TRUE;                 // Successfully set interval
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

    hodr_setAcquisitionMode((int)mode);                               // Set the acquisition mode in HODR
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
            fflush(stdout); // Flush stdout to ensure immediate output
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


            endThread = true;       // Set the flag to end the command thread

            g_main_loop_quit(loop); // Quit the main loop
  

            return FALSE;   // Do not schedule power on if overheat reboot count exceeded
            

        }

        g_timeout_add_seconds(280, (GSourceFunc)powerOnHodr, control); // Schedule power on after 5 minutes
        g_timeout_add_seconds(300, (GSourceFunc)activateHodr, control);

        

        return FALSE; // Stop further temperature updates
    }

    int tempStatus;
    hodr_getCurrentTemperatureStatus(&tempStatus); // Get current temperature status

    
    int status;
    hodr_getStatus(&status); // Get the current acquisition status from HOD
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

static gboolean StartAcquisitionIfPossible(gpointer controlPtr)
{
    Control *control = (Control *)controlPtr;

    printf("Attempting to start acquisition...\n");
    fflush(stdout); // Flush stdout to ensure immediate output
    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not starting acquisition.\n");

        return false; // Do not start acquisition if Andor SDK is not active
    }



    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    hodr_setAcquisitionMode(AC_ACQMODE_SINGLE);               // Set acquisition mode to 1 (assuming this is a valid mode)
    control_set_acquisition_mode(control, AC_ACQMODE_SINGLE); // Set the acquisition mode in the control object

    
    unsigned int result = hodr_startAcquisition(); // Start acquisition in HODR

    
    int status;
    hodr_getStatus(&status); // Get the current acquisition status from HOD
    control_set_acquisition_status(control, status); // Set the acquisition status in the control object
    
    pthread_mutex_unlock(&lock);                   // Unlock the mutex after starting acquisition
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to start acquisition: %d\n", result);
    } else
    {
        printf("Acquisition started successfully.\n");
        fflush(stdout); // Flush stdout to ensure immediate output

        nTriggeredSpectra++;
        return true; // Successfully started acquisition
    }

    return false; // Failed to start acquisition
}

static gboolean StartAcquisitionIfPossibleWallclock(gpointer controlPtr)
{
    Control *control = (Control *)controlPtr;

    printf("Attempting to start acquisition in wallclock mode...\n");
    fflush(stdout); // Flush stdout to ensure immediate output
    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not starting acquisition.\n");

        return false; // Do not start acquisition if Andor SDK is not active
    }


    if (!control_get_wallclock_acquisition_active(control)) // Check if wallclock acquisition is active
    {
        printf("Wallclock acquisition is not active. Not starting acquisition.\n");
        fflush(stdout); // Flush stdout to ensure immediate output
        return false; // Do not start acquisition if wallclock acquisition is not active
    }

    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    hodr_setAcquisitionMode(AC_ACQMODE_SINGLE);               // Set acquisition mode to 1 (assuming this is a valid mode)
    control_set_acquisition_mode(control, AC_ACQMODE_SINGLE); // Set the acquisition mode in the control object

    //g_source_remove(timerHandle);                 // Remove the existing timer to prevent multiple acquisitions

    unsigned int result = hodr_startAcquisition(); // Start acquisition in HODR

    
    int status;
    hodr_getStatus(&status); // Get the current acquisition status from HOD
    control_set_acquisition_status(control, status); // Set the acquisition status in the control object

    pthread_mutex_unlock(&lock);                   // Unlock the mutex after starting acquisition
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to start acquisition: %d\n", result);
    } else
    {
        printf("Acquisition started successfully in wallclock mode.\n");
        fflush(stdout); // Flush stdout to ensure immediate output

        nTriggeredSpectra++;
        return true; // Successfully started acquisition
    }

    return false; // Failed to start acquisition
}


static gboolean db_startAcquisition(Control *control, GDBusMethodInvocation *invocation, gdouble integration_time, gboolean pre_amp_gain, gdouble interval_time, guint mode, guint number, gpointer)
{

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not starting acquisition.\n");

        control_complete_start_acquisition(control, invocation, 0); // Complete the D-Bus method invocation with failure
        return TRUE;                                                // Do not start acquisition if Andor SDK is not active
    }
    printf("Starting acquisition with integration time: %.9f seconds\n", integration_time);
    printf("Waiting to acquire lock for starting acquisition...\n");
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    printf("Acquired lock for starting acquisition.\n");

    if (integration_time > 0)
    {
        printf("Setting exposure time to %.9f seconds.\n", integration_time);
        hodr_setExposureTime(integration_time);                       // Set exposure time in seconds
        control_set_integration_time_secs(control, integration_time); // Set integration time in the control object
    }

    int readmode = hodr_getReadMode(); // Get the current read mode from HOD
    printf("Current read mode: %d\n", readmode);


    control_set_pre_amp_gain(control, pre_amp_gain); // Set preamp gain in the control object
    hodr_setPreAmpGain((int)pre_amp_gain); // Set preamp gain in HODR



    if (mode > 5) // Assuming valid modes are 0, 1, and 2
    {
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT, "Invalid acquisition mode: %d", mode);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return FALSE;                // Invalid acquisition mode
    }
    else if (mode != 0)
    {
        hodr_setAcquisitionMode((int)mode); // Set the acquisition mode in HODR
        // control_set_acquisition_mode(control, mode); // Set the acquisition mode in the control object
    }

    if (interval_time >= 0)
    {
        printf("Setting kinetic cycle time to %.2f seconds.\n", interval_time);
        hodr_setKineticCycleTime(interval_time); // Set kinetic cycle time in seconds
    }

    if (number > 0)
    {
        printf("Setting number of captures to %d.\n", number);
        hodr_setNumberKinetics(number); // Set the number of accumulations in HODR
        // control_set_number_spectra(control, (uint32_t)number); // Set the number of accumulations in the control object
    }

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
    hodr_getStatus(&status); // Get the current acquisition status from HOD
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

static gboolean db_getLastSpectrum(Control *control, GDBusMethodInvocation *invocation, gpointer)
{
    printf("Requesting last captured spectrum data...\n");

    if (nCapturedSpectra == 0)
    {
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_NOT_FOUND, "No spectra captured yet.");

        return FALSE; // No spectra captured yet
    }

    // Get last line from the data file

    printf("Waiting to acquire lock for reading data file...\n");
    fflush(stdout);
    pthread_mutex_lock(&dataFileLock); // Lock the mutex for data file operations
    printf("Acquired lock for reading data file.\n");
    fflush(stdout);
    
    int status;
    hodr_getStatus(&status); // Get the current acquisition status from HOD
    control_set_acquisition_status(control, status); // Set the acquisition status in the control object

    FILE *file = fopen(outFile, "r");
    if (file == NULL)
    {
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_FAILED, "Failed to open data file: %s", outFile);
        pthread_mutex_unlock(&dataFileLock); // Unlock the mutex before returning
        pthread_mutex_unlock(&lock);         // Unlock the main lock before returning
        printf("Failed to open data file: %s\n", outFile);
        printf("Unlocked mutex after failed file open.\n");
        fflush(stdout);
        return FALSE; // Error opening data file
    }

    char line[8192];     // Buffer for reading lines from the file
    char nextLine[8192]; // Buffer for the next line
    int nLines = 0;      // Counter for the number of lines read
    while (fgets(line, sizeof(line), file) != NULL)
    {
        nLines++; // Increment the line counter
        // Store the current line in nextLine
        strncpy(nextLine, line, sizeof(nextLine) - 1);
        nextLine[sizeof(nextLine) - 1] = '\0'; // Ensure null termination
    }

    fclose(file);                        // Close the file after reading
    pthread_mutex_unlock(&dataFileLock); // Unlock the mutex after reading the data file
    printf("Unlocked mutex after reading data file.\n");
    printf("Read %d lines from the data file.\n", nLines); // Log the number of lines read
    fflush(stdout);

    // second token is timestamp, convert from string to unix timestamp
    printf("Parsing timestamp...\n");
    fflush(stdout);
    // First token is spectrum ID, ignore it
    char token[64];



    char *timeToken = strtok(line, ",");

    strcpy(token, timeToken); // Copy the token to a buffer

    double exposureTimeDouble;

    // third token is exposure time as float.

    // convert from string to float then multiply by 1e9 to convert to nanoseconds
    char *exposureTimetoken = strtok(NULL, ",");
    strcpy(token, exposureTimetoken); // Copy the token to a buffer

    exposureTimeDouble = atof(token);

    //fourth token is preamp gain as float
    double preAmpGainDouble;
    char *preAmpGainToken = strtok(NULL, ",");
    printf("Preamp Gain token: %s\n", preAmpGainToken); // Log the preamp gain token
    strcpy(token, preAmpGainToken);                     // Copy the token to a buffer
    preAmpGainDouble = atof(token);
    printf("Preamp Gain parsed: %.2f\n", preAmpGainDouble); // Log the parsed preamp gain

    double temperatureDouble;

    // fifth token is temperature as float
    char *temperatureToken = strtok(NULL, ",");

    printf("Temperature token: %s\n", temperatureToken); // Log the temperature token
    strcpy(token, temperatureToken);                     // Copy the token to a buffer

    temperatureDouble = atof(token);
    printf("Temperature parsed: %.2f degrees Celsius\n", temperatureDouble); // Log the parsed temperature

    printf("Initializing GVariant builder...\n");
    GVariantBuilder *builder;
    builder = g_variant_builder_new(G_VARIANT_TYPE("ai")); // Create a new GVariant builder for an array of integers

    printf("Parsing data values...\n");
    // remaining tokens are data values, split by comma


    // Get the rest of the line and split by comma
    char *dataToken;
    int dataCount = 0; // Count of data values
    while ((dataToken = strtok(NULL, ",")) != NULL)
    {
        dataCount++;                                    // Increment the data count
        int32_t dataValue = (int32_t)atoi(dataToken);   // Convert the token to an integer
        g_variant_builder_add(builder, "i", dataValue); // Add the integer to the GVariant builder
    }

    printf("Parsed %d data values.\n", dataCount); // Log the number of data values parsed

    // g_dbus_method_invocation_return_value(invocation, value); // Return the GVariant as the response

    printf("Creating response GVariant...\n");
    GVariant *response;
    response = g_variant_new("(sdddai)", timeToken, exposureTimeDouble, preAmpGainDouble, temperatureDouble, builder); // Create a response GVariant with timestamp, exposure time, temperature, and data array

    printf("Completing D-Bus method invocation...\n");
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
    //guint wallclockInterval = interval; // Set the wallclock acquisition interval

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
    g_source_remove(timerHandle);                             // Remove the timer source to stop wallclock acquisition
    timerHandle = 0;                                          // Reset the timer handle

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

int appendToFile(float *exposureTime, float *preAmpGain, double *temperature, int *readMode, int *singleTrackHeight, int *singleTrackCentre, const int32_t *data, size_t size)
{

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
    // current_time -= (time_t)(*exposureTime * 1000000000);
    time_info = localtime(&current_time);

    strftime(timeString, sizeof(timeString), "%Y-%m-%dT%H:%M:%S", time_info);
    // fprintf(file, "%d,", spectrumID);      // Write spectrum ID
    fprintf(file, "%s,", timeString);      // Write timestamp
    fprintf(file, "%.5f,", *exposureTime); // Write exposure time
    fprintf(file, "%.1f,", *preAmpGain);    // Write pre-amplifier gain
    fprintf(file, "%.2f,", *temperature);  // Write temperature
    fprintf(file, "%d,", *readMode);         // Write read mode
    if (*readMode == READ_MODE_SINGLE_TRACK)
    {
        fprintf(file, "%d,", *singleTrackHeight); // Write single track height if in single track mode
        fprintf(file, "%d,", *singleTrackCentre); // Write single track centre if in single track mode
    } else
    {
        fprintf(file, ",,"); // Write empty values for single track height and centre if not in single track mode
    }
    for (size_t i = 0; i < size; i++)
    {
        fprintf(file, "%d", data[i]);
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

void adjustIntegrationTime(Control *control, unsigned int targetIntensity, float exposureTime, int32_t *data, size_t size, unsigned int attemptLimit)
{

    int32_t maxIntensity; // Variable to hold the maximum intensity found in the data

    float previousIntegrationTime = 0; // Store the previous integration time for comparison

    while (true)
    {
         

        if (targetIntensity == 0)
        {
            printf("Target intensity is not set or invalid. Skipping adjustment of integration time.\n");
            fflush(stdout);
            return; // Do not adjust if target intensity is not set or invalid
        }

        printf("Adjusting integration time based on target intensity: %d\n", targetIntensity);
        fflush(stdout);
        maxIntensity = data[0]; // Initialize max intensity with the first data point
        for (size_t i = 1; i < size; i++)
        {
            if (data[i] > maxIntensity)
            {
                maxIntensity = data[i]; // Update max intensity if current data is greater
            }
        }

        printf("Max intensity from data: %d at integration time %.6f\n", maxIntensity, exposureTime);
        printf("Max integration time: %.6f seconds, Min integration time: %.6f seconds\n", maximumIntegrationTime, minimumIntegrationTime);
        float newIntegrationTime = exposureTime; // Initialize new integration time with the current exposure time
        if (maxIntensity >= 65534)
        {
            newIntegrationTime *= 0.5; // Reduce integration time by half if max intensity is too high
            newIntegrationTime = (newIntegrationTime < minimumIntegrationTime) ? minimumIntegrationTime : newIntegrationTime; // Ensure new integration time is not below minimum
            if (exposureTime <= minimumIntegrationTime)
            {

                hodr_setReadMode(READ_MODE_SINGLE_TRACK); // Set read mode to 2  Single Track(assuming this is a valid mode)
                

                int centre, height;

                int readMode =hodr_getReadMode(); // Get the current read mode from HODR
                hodr_getSingleTrack(&centre, &height); // Get the current Single Track settings



                printf("Integration time is already at minimum. Cannot reduce further.\n");
                printf("Setting read mode to Single Track with centre at %d and height of %d\n", centre, height);
                printf("Current read mode: %d\n", readMode);
                fflush(stdout);
            }
            printf("Max intensity too high, reducing integration time to %.6f seconds\n", newIntegrationTime);
            fflush(stdout);

        }
        else
        {

            float ratio = (float)maxIntensity / (float)targetIntensity; // Calculate ratio of max intensity to target intensity
            if (ratio > 0.95 && ratio < 1.05)
            {
                printf("Max intensity is within 10%% of target intensity, keeping current integration time: %.6f seconds\n", newIntegrationTime);
                fflush(stdout);
                return; // If max intensity is within 10% of target intensity, keep the current integration time
            }
            newIntegrationTime = exposureTime * ((float)targetIntensity / (float)maxIntensity); // Adjust integration time based on target intensity

            int readMode = hodr_getReadMode(); // Get the current read mode from HODR
            printf("Current read mode: %d\n", readMode);
            if (newIntegrationTime > 1.5 * exposureTime && readMode == READ_MODE_SINGLE_TRACK) // If new integration time is significantly higher and read mode is Full Vertical Binning
            {
                printf("Max intensity too low, changing read mode to Full Vertical Binning to increase sensitivity.\n");
                hodr_setReadMode(READ_MODE_FVB); // Set read mode to 1 Full Vertical Binning (assuming this is a valid mode)
                int newReadMode =  hodr_getReadMode(); // Get the new read mode from HODR
                printf("New read mode: %d\n", newReadMode);
            }

            if (newIntegrationTime >= maximumIntegrationTime)
            {
             printf("Max integration time reached\n");
             attemptLimit = 1;
             newIntegrationTime = maximumIntegrationTime;
            } else if (newIntegrationTime <= minimumIntegrationTime) {
             printf("Min integration time reached\n");
             attemptLimit = 1;
             newIntegrationTime = minimumIntegrationTime;
            } else if (exposureTime == previousIntegrationTime) {
             printf("Integration time unchanged, must be at minimum, stopping adjustment\n");
             attemptLimit = 1;
            }
	        printf("Adjusted integration time based on target intensity: %.6f seconds\n", newIntegrationTime);
            fflush(stdout);
        }

        //hodr_abortAcquisition();

        printf("Setting new integration time: %.6f seconds\n", newIntegrationTime);
        unsigned int result = hodr_setExposureTime(newIntegrationTime); // Set the new exposure time in HODR
        float exposureTimeResult, kineticCycleTime, readoutTime;

        hodr_getAcquisitionTimings(&exposureTimeResult, &kineticCycleTime, &readoutTime); // Get acquisition timings

        if (exposureTimeResult / newIntegrationTime > 1.05) 
        {
            printf("Warning: Actual exposure time %.6f seconds differs from requested %.6f seconds by more than 5%%\n", exposureTimeResult, newIntegrationTime);
            fflush(stdout);
            printf("Attempting to set correct exposure time by subtracting minimum integration time: %.6f seconds\n", newIntegrationTime - minimumIntegrationTime);
            hodr_setExposureTime(newIntegrationTime - minimumIntegrationTime); // Set the exposure time to the actual value returned by HODR
            hodr_getAcquisitionTimings(&exposureTimeResult, &kineticCycleTime, &readoutTime); // Get acquisition timings again
            printf("New exposure time after adjustment: %.6f seconds\n", exposureTimeResult);
        }

        
        result = hodr_startAcquisition(); // Restart acquisition with the new exposure time
        if (result != DRV_SUCCESS)
        {
            fprintf(stderr, "Failed to restart acquisition with new integration time: %d\n", result);
            return; // Error restarting acquisition
        }

        

        result = WaitForAcquisition(); // Wait for acquisition to finish before checking again


        previousIntegrationTime = exposureTime; // Store the previous integration time for comparison
        
        if (result != DRV_SUCCESS)
        {
            fprintf(stderr, "Error waiting for acquisition: %d\n", result);
            return; // Error waiting for acquisition
        }



        
        result = hodr_getMostRecentImage(data, (size_t)xpixels); // Get the most recent image acquired
        
        int readMode = hodr_getReadMode(); // Get the current read mode from HODR
        int centre, height;
        hodr_getSingleTrack(&centre, &height); // Get the current Single Track settings

        hodr_getAcquisitionTimings(&exposureTime, &kineticCycleTime, &readoutTime); // Get acquisition timings

        int preAmpGainIndex;
        char preAmpGainString[32]; // Buffer for pre-amplifier gain string
        hodr_getCurrentPreAmpGain(&preAmpGainIndex, preAmpGainString); // Get the pre-amplifier gain index
        float preAmpGain = preAmpGainValues[preAmpGainIndex]; // Get the pre-amplifier gain value based on the index


        result = appendToFile(&exposureTime, &preAmpGain, &lastTemperature, &readMode, &centre, &height, data, xpixels); // Append data to output file



        control_set_number_spectra(control, nCapturedSpectra);

        control_emit_acquisition_finished(control, nCapturedSpectra + 1);
	    attemptLimit -= 1;
        printf("Attempts: %d\n", attemptLimit);
        if (attemptLimit <= 0) // Check if attempt limit is reached
        {
            printf("Attempt limit reached. Stopping adjustment of integration time.\n");
            fflush(stdout);
            return; // Stop adjusting if attempt limit is reached
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

    guint wallclockInterval = control_get_wallclock_interval(control); // Get the wallclock interval from the control object
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
        fflush(stdout); // Flush stdout to ensure immediate output
        StartAcquisitionIfPossibleWallclock(control); // Start acquisition immediately if seconds is 0
        timeoutID = 0;                       // Reset the timeout ID since acquisition starts immediately

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
        hodr_getStatus(&status); // Get the current acquisition status from HOD
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
        float exposureTime, kineticCycleTime, readoutTime;
        hodr_getAcquisitionTimings(&exposureTime, &kineticCycleTime, &readoutTime); // Get acquisition timings
        printf("Acq. %d: Acquisition timings - Exposure: %.6f, Kinetic Cycle: %.6f, Readout: %.6f\n", nCapturedSpectra + 1, exposureTime, kineticCycleTime, readoutTime);

        int preAmpGainIndex;
        char preAmpGainString[32]; // Buffer for pre-amplifier gain string

        hodr_getCurrentPreAmpGain(&preAmpGainIndex, preAmpGainString); // Get the pre-amplifier gain index
        float preAmpGain = preAmpGainValues[preAmpGainIndex]; // Get the pre-amplifier gain value based on the index

        int readMode = hodr_getReadMode(); // Get the current read mode from HODR
        int centre, height;
        hodr_getSingleTrack(&centre, &height); // Get the current Single Track settings


        fflush(stdout);
        printf("Acq. %d: Last temperature: %.2f\n", nCapturedSpectra + 1, lastTemperature); // Log the last temperature
        fflush(stdout);
        result = appendToFile(&exposureTime, &preAmpGain, &lastTemperature, &readMode, &centre, &height, data, xpixels); // Append data to output file
        control_set_number_spectra(control, nCapturedSpectra);
        unsigned int targetIntensity = control_get_target_intensity(control); // Get the target intensity from the control object
        printf("Target intensity: %d\n", targetIntensity);                    // Log the target intensity
        fflush(stdout);

        if (targetIntensity > 0)
        {
            printf("Acq. %d: Target intensity: %d at integration time %.5fs\n", nCapturedSpectra + 1, targetIntensity, exposureTime); // Log the target intensity
            fflush(stdout);
            adjustIntegrationTime(control, targetIntensity, exposureTime, data, xpixels, adjustmentAttemptLimit); // Adjust integration time based on target intensity
        }
        control_set_number_spectra(control, nCapturedSpectra);
        //hodr_getAcquisitionTimings(&exposureTime, &kineticCycleTime, &readoutTime);     // Get acquisition timings

        control_set_data_path(control, outFile);   // Set the data path in the control object
        
        setNextAcquisition(&timerHandle, control); // Set the next acquisition time based on the wallclock acquisition interval
        printf("Data appended to file. Result: %d, N captured spectra: %d\n", result, nCapturedSpectra);
        fflush(stdout);
        pthread_mutex_unlock(&lock); // Unlock the mutex after processing
        printf("Acq. %d: Unlocked mutex after processing acquired data.\n", nCapturedSpectra);
        fflush(stdout);
    }

    return NULL; // Return NULL to indicate the thread has finished
}
