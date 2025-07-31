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

pthread_mutex_t lock;
pthread_mutex_t endThreadLock;
pthread_mutex_t dataFileLock;        // Mutex for data file operations
pthread_mutex_t acquisitionLoopLock; // Mutex for acquisition loop operations
bool endThread = false;              // Flag to signal the command thread to end

unsigned int initialTargetIntensity = 60000; // Target intensity for the acquisition
int32_t initialTargetTemperature = -70;      // Initial target temperature for the acquisition

char andorFile[256] = "../miniforge3/pkgs/andor2-sdk-2.104.30064-0/etc/andor/";
char outFile[256]; // Output file for data

int countLines();
int createDataFile(char *directory, char *filename);
int appendToFile(const char *filename, float *exposureTime, double *temperature, const int32_t *data, size_t size);
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
static gboolean db_stopLive(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_exitMainLoop(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_startAcquisition(Control *control, GDBusMethodInvocation *invocation, gdouble integration_time, gdouble interval_time, guint mode, guint number, gpointer user_data);
static gboolean db_setAcquisitionMode(Control *control, GDBusMethodInvocation *invocation, guint32 mode, gpointer user_data);
static gboolean db_stopAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_setInterval(Control *control, GDBusMethodInvocation *invocation, gdouble interval, gpointer user_data);
static gboolean db_getLastSpectrum(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_setTargetIntensity(Control *control, GDBusMethodInvocation *invocation, guint intensity, gpointer user_data);
static gboolean db_startWallclockAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_stopWallclockAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer user_data);
static gboolean db_setWallclockAcquisitionInterval(Control *control, GDBusMethodInvocation *invocation, guint interval, gpointer user_data);

static gboolean StartAcquisitionIfPossible(gpointer controlPtr);

void setNextAcquisition(int *timeoutID, Control *control); // Function to set the next acquisition
// static gboolean db_getData(Control *control, GDBusMethodInvocation *invocation, gint ref, gpointer user_data);

void *handleAcquisitionLoop();

char dataDir[256] = "../hodr_data"; // Directory for data files

double lastTemperature = 0.0;       // Last temperature read from the device
double lastTargetTemperature = 0.0; // Last target temperature set
int lastTemperatureStatus = 0;      // Last temperature status read from the device

uint32_t nTriggeredSpectra = 0; // Number of triggered spectra
uint32_t nCapturedSpectra = 0;  // Number of captured spectra

int xpixels, ypixels; // Detector size
GMainLoop *loop;

guint dataWaitFunctionRef;

HODR_Config_t *hodr_cfg; // Pointer to HODR configuration structure

bool andorActive = false; // Flag to indicate if Andor SDK is active
bool overHeated = false;  // Flag to indicate if the device is overheated

u_int32_t wallclockInterval = 5;        // Interval for wallclock acquisition in minutes (0 means no schedule).
bool wallclockAcquisitionActive = true; // Flag to indicate if wallclock acquisition is active
// Must be a factor of 60 to ensure proper timing repeating every hour.
int timerHandle;

pthread_t acqThread;

void signalHandler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM)
    {
        printf("Received signal %d, exiting...\n", signal);

        fflush(stdout);         // Flush stdout to ensure all output is printed
        endThread = true;       // Set the flag to end the command thread
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

    // if (hodr_init(hodr_cfg, andorFile, outFile, true) != DRV_SUCCESS)
    // {
    //     fprintf(stderr, "Failed to initialize HODR.\n");
    // }
    // else
    // {
    //     printf("HODR initialized successfully.\n");
    //     andorActive = true;             // Set Andor SDK active flag
    //     hodr_setCoolerMode(true);       // Turn on the cooler
    //     hodr_setTargetTemperature(-70); // Set target temperature to -70 degrees Celsius

    //     hodr_getDetectorSize(&xpixels, &ypixels); // Get detector size

    //     float currentTemp;
    //     hodr_getCurrentTemperatureFloat(&currentTemp); // Get current temperature

    //     int minTemp, maxTemp;
    //     hodr_getTemperatureRange(&minTemp, &maxTemp); // Get temperature range
    // }

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
    g_signal_connect(control, "handle-stop_acquisition", G_CALLBACK(db_stopAcquisition), NULL);                       // Connect the signal for stopping acquisition
    g_signal_connect(control, "handle-set_target_intensity", G_CALLBACK(db_setTargetIntensity), NULL);                // Connect the signal for setting target intensity
    g_signal_connect(control, "handle-set_interval", G_CALLBACK(db_setInterval), NULL);                               // Connect the signal for setting interval
    g_signal_connect(control, "handle-stop_live", G_CALLBACK(db_stopLive), NULL);                                     // Connect the signal for stopping live mode
    g_signal_connect(control, "handle-get_data", G_CALLBACK(db_getLastSpectrum), NULL);                               // Connect the signal for getting data
    g_signal_connect(control, "handle-exit", G_CALLBACK(db_exitMainLoop), NULL);                                      // Connect the signal for exiting the application

    int nSpectra = countLines();
    printf("Number of spectra in data file: %d\n", nSpectra);
    fflush(stdout); // Flush stdout to ensure immediate output
    if (nSpectra >= 0)
        nCapturedSpectra = (uint32_t)nSpectra;
    control_set_number_spectra(control, nCapturedSpectra); // Initialize number of spectra to 0
    control_set_data_path(control, outFile);               // Set the data path in the control object
    control_set_wallclock_next_capture(control, 0);        // Initialize wallclock next capture to 0

    control_set_over_heat_reboot_count(control, 0);             // Initialize overheat reboot count to 0
    control_set_temperature(control, initialTargetTemperature); // Set the initial target temperature in the control object

    control_set_target_intensity(control, initialTargetIntensity);                 // Set the target intensity in the control object
    control_set_target_temperature(control, initialTargetTemperature);             // Set the target temperature in the control object
    control_set_wallclock_interval(control, wallclockInterval);                    // Set wallclock acquisition interval in the control object
    control_set_wallclock_acquisition_active(control, wallclockAcquisitionActive); // Set wallclock acquisition active status in the control object
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
    unsigned int result = hodr_init(hodr_cfg, andorFile, outFile, false); // Initialize HODR
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
        g_timeout_add_seconds(1, (GSourceFunc)db_getTemperature, control); // Schedule next temperature check
        hodr_getDetectorSize(&xpixels, &ypixels);                          // Get detector size
        printf("Detector size: %d x %d pixels\n", xpixels, ypixels);

        if (control_get_wallclock_acquisition_active(control)) // Check if wallclock acquisition is active
        {
            printf("Wallclock acquisition is active. Setting next acquisition...\n");
            setNextAcquisition(&timerHandle, control); // Set the next acquisition
        }
        else
        {
            printf("Wallclock acquisition is not active.\n");
        }

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

    printf("Activating HODR...\n");
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

    unsigned int initResult = hodr_init(hodr_cfg, andorFile, outFile, true); // Reinitialize HODR
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

static gboolean db_updateNCaptures(gpointer control)
{
    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Skipping update of number of captures.\n");
        return TRUE; // Do not update if Andor SDK is not active
    }

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
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    if (int_time <= 0)
    {
        g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT, "Invalid integration time: %.9f", int_time);
        pthread_mutex_unlock(&lock); // Unlock the mutex before returning
        return TRUE;                 // Invalid integration time
    }

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
    control_complete_set_integration_time(control, invocation, TRUE); // Complete the D-Bus method invocation with success
    printf("Integration time set to %.9f seconds successfully.\n", int_time);
    pthread_mutex_unlock(&lock); // Unlock the mutex after setting the integration time
    return TRUE;                 // Successfully set integration time
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
        printf("Andor SDK is not active. Not getting temperature.\n");
        return TRUE; // Do not update if Andor SDK is not active
    }
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

    if (!andorActive) // Check if Andor SDK is active
    {
        printf("Andor SDK is not active. Not starting acquisition.\n");

        return false; // Do not start acquisition if Andor SDK is not active
    }

    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety

    hodr_setAcquisitionMode(AC_ACQMODE_SINGLE);               // Set acquisition mode to 1 (assuming this is a valid mode)
    control_set_acquisition_mode(control, AC_ACQMODE_SINGLE); // Set the acquisition mode in the control object

    pthread_mutex_unlock(&lock);                   // Unlock the mutex before starting acquisition
    unsigned int result = hodr_startAcquisition(); // Start acquisition in HODR

    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to start acquisition: %d\n", result);
    }

    return false; // Failed to start acquisition
}

static gboolean db_startAcquisition(Control *control, GDBusMethodInvocation *invocation, gdouble integration_time, gdouble interval_time, guint mode, guint number, gpointer)
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

    unsigned int result = hodr_startAcquisition(); // Start acquisition in HODR
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

    // hodr_startAcquisitionOnceTemperatureStabilized(); // Start acquisition once temperature is stabilized
    //  generate a new spectrum ID
    uint32_t spectrumID = nTriggeredSpectra++; // Increment the captured spectra count to generate a new spectrum ID

    pthread_mutex_unlock(&lock); // Unlock the mutex after starting acquisition
    printf("Unlocked mutex after starting acquisition.\n");
    printf("New spectrum ID generated: %u\n", spectrumID);

    control_complete_start_acquisition(control, invocation, spectrumID); // Complete the D-Bus method invocation

    // printf("Data waiting loop started with function reference: %u\n", dataWaitFunctionRef);
    // printf("Initiated data waiting loop.\n");
    return TRUE; // Successfully started acquisition
}

static gboolean db_stopAcquisition(Control *control, GDBusMethodInvocation *invocation, gpointer)
{
    printf("Stopping acquisition...\n");
    pthread_mutex_lock(&lock); // Lock the mutex to ensure thread safety
    printf("Acquired lock for stopping acquisition.\n");
    unsigned int result = hodr_abortAcquisition(); // Abort acquisition in HODR
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

    // char *firstToken = strtok(line, ","); // Get the first token (spectrum ID)
    //  if (firstToken == NULL)
    //  {
    //      g_dbus_method_invocation_return_error(invocation, G_IO_ERROR, G_IO_ERROR_NOT_FOUND, "Failed to parse spectrum ID from data file.");
    //      printf("Failed to parse spectrum ID from data file.\n");
    //      return FALSE; // Error parsing spectrum ID
    //  }

    char *timeToken = strtok(line, ",");

    strcpy(token, timeToken); // Copy the token to a buffer

    double exposureTimeDouble;

    // third token is exposure time as float.

    // convert from string to float then multiply by 1e9 to convert to nanoseconds
    char *exposureTimetoken = strtok(NULL, ",");
    strcpy(token, exposureTimetoken); // Copy the token to a buffer

    exposureTimeDouble = atof(token);

    double temperatureDouble;

    // fourth token is temperature as float
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

    // int32_t dataArray[xpixels]; // Array to hold data values
    // int dataArrayCount = 0; // Count of data values

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
    response = g_variant_new("(sddai)", timeToken, exposureTimeDouble, temperatureDouble, builder); // Create a response GVariant with timestamp, exposure time, temperature, and data array

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
    wallclockInterval = interval; // Set the wallclock acquisition interval

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

int appendToFile(const char *filename, float *exposureTime, double *temperature, const int32_t *data, size_t size)
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
    fprintf(file, "%.9f,", *exposureTime); // Write exposure time
    fprintf(file, "%.2f,", *temperature);  // Write temperature
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

void adjustIntegrationTime(unsigned int targetIntensity, float exposureTime, int32_t *data, size_t size, unsigned int attemptLimit)
{

    int32_t maxIntensity; // Variable to hold the maximum intensity found in the data

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

        printf("Max intensity from data: %d\n", maxIntensity);
        float newIntegrationTime = exposureTime; // Initialize new integration time with the current exposure time
        if (maxIntensity >= 65534)
        {
            newIntegrationTime *= 0.5; // Reduce integration time by half if max intensity is too high
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
            printf("Adjusted integration time based on target intensity: %.6f seconds\n", newIntegrationTime);
            fflush(stdout);
        }

        hodr_abortAcquisition();
        unsigned int result = hodr_setExposureTime(newIntegrationTime); // Set the new exposure time in HODR

        result = hodr_startAcquisition(); // Restart acquisition with the new exposure time
        if (result != DRV_SUCCESS)
        {
            fprintf(stderr, "Failed to restart acquisition with new integration time: %d\n", result);
        }

        if (attemptLimit-- == 0) // Check if attempt limit is reached
        {
            printf("Attempt limit reached. Stopping adjustment of integration time.\n");
            fflush(stdout);
            return; // Stop adjusting if attempt limit is reached
        }

        result = WaitForAcquisition(); // Wait for acquisition to finish before checking again
        if (result != DRV_SUCCESS)
        {
            fprintf(stderr, "Error waiting for acquisition: %d\n", result);
            return; // Error waiting for acquisition
        }
        result = hodr_getMostRecentImage(data, (size_t)xpixels); // Get the most recent image acquired
        float kineticCycleTime, readoutTime;
        hodr_getAcquisitionTimings(&exposureTime, &kineticCycleTime, &readoutTime); // Get acquisition timings
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

    wallclockInterval = control_get_wallclock_interval(control); // Get the wallclock interval from the control object
    printf("Current wallclock interval: %d minutes\n", wallclockInterval);
    fflush(stdout); // Flush stdout to ensure immediate output
    if (wallclockInterval == 0)
    {
        printf("Acquisition schedule interval is not set. Cannot set next acquisition.\n");
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
        StartAcquisitionIfPossible(control); // Start acquisition immediately if seconds is 0
        timeoutID = 0;                       // Reset the timeout ID since acquisition starts immediately

        return; // Do not set a timeout if acquisition starts immediately
    }

    printf("Setting next acquisition in %u seconds...\n", seconds);
    fflush(stdout);

    *timeoutID = g_timeout_add_seconds(seconds, (GSourceFunc)StartAcquisitionIfPossible, control); // Set a timeout to handle the acquisition loop

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
        fflush(stdout);
        printf("Acq. %d: Last temperature: %.2f\n", nCapturedSpectra + 1, lastTemperature); // Log the last temperature
        fflush(stdout);

        unsigned int targetIntensity = control_get_target_intensity(control); // Get the target intensity from the control object
        printf("Target intensity: %d\n", targetIntensity);                    // Log the target intensity
        fflush(stdout);

        if (targetIntensity > 0)
        {
            printf("Acq. %d: Target intensity: %d at integration time %.5fs\n", nCapturedSpectra + 1, targetIntensity, exposureTime); // Log the target intensity
            fflush(stdout);
            adjustIntegrationTime(targetIntensity, exposureTime, data, xpixels, 5); // Adjust integration time based on target intensity
        }

        hodr_getAcquisitionTimings(&exposureTime, &kineticCycleTime, &readoutTime);     // Get acquisition timings
        result = appendToFile(outFile, &exposureTime, &lastTemperature, data, xpixels); // Append data to output file
        control_set_number_spectra(control, nCapturedSpectra);
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
