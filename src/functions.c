#include "hodr.h"
#include "atmcdLXd.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <stdbool.h>


HODR_Config_t cfg = {0}; // Default configuration

unsigned int hodr_init(HODR_Config_t *config, char *andorPath, const char *outFile, bool resetConfig)
{
    
    strncpy(cfg.OUT_FILE, outFile, sizeof(cfg.OUT_FILE) - 1);
    cfg.OUT_FILE[sizeof(cfg.OUT_FILE) - 1] = '\0'; // Ensure null termination

    // Initialize the Andor SDK
    printf("Initializing Andor SDK with path: %s\n", andorPath);

    int result = Initialize(andorPath);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to initialize Andor SDK: %d\n", result);
        return result; // Error
    }

    if (cfg.ACQUISITION_MODE == 0 || resetConfig) // Check if configuration is not initialized
    {
        cfg = (HODR_Config_t){0}; // Reset configuration to default values

        // Set default configuration
        cfg.READ_MODE = READ_MODE_FVB;              // Default read mode
        cfg.SHUTTER_TYPE = SHUTTER_TYP_OPEN_LOW;    // Default shutter type
        cfg.SHUTTER_MODE = SHUTTER_MODE_FULLY_AUTO; // Default shutter mode
        cfg.ACQUISITION_MODE = 1;                   // Default acquisition mode
        cfg.SERIES_LENGTH = 5;                      // Default series length
        cfg.NUMBER_ACQUISITIONS = 0;                // Default number of acquisitions
        cfg.NUMBER_ACCUMULATIONS = 1;               // Default number of accumulations
        cfg.INTERVAL = 1.0f;                        // Default interval in seconds
        cfg.INTEGRATION_TIME = 0.01f;               // Default integration time in seconds
        cfg.ACQ_FLAG = false;                       // Acquisition flag
        strncpy(cfg.OUT_FILE, outFile, sizeof(cfg.OUT_FILE) - 1);

      
    } 

    GetDetector(&cfg.xpixels, &cfg.ypixels);               // Get detector size
    hodr_setNumberAccumulations(cfg.NUMBER_ACCUMULATIONS); // Set number of accumulations
    hodr_setAcquisitionMode(cfg.ACQUISITION_MODE);
    hodr_setNumberKinetics(cfg.SERIES_LENGTH);  // Set number of kinetics
    hodr_setKineticCycleTime(cfg.INTERVAL);     // Set kinetic cycle time
    hodr_setExposureTime(cfg.INTEGRATION_TIME); // Set integration time
    hodr_setReadMode(cfg.READ_MODE);
    hodr_setShutter(cfg.SHUTTER_TYPE, cfg.SHUTTER_MODE, 0, 0); // Set shutter to fully auto mode
    printf("Andor SDK initialized successfully.\n");

    config = &cfg; // Update the provided configuration pointer
    return DRV_SUCCESS; // Success
}

unsigned int hodr_deinit()
{
    unsigned int result = ShutDown();
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to shut down Andor SDK: %d\n", result);
        return result; // Error
    }
    printf("Andor SDK shut down successfully.\n");
    return DRV_SUCCESS; // Success
}

unsigned int hodr_getDetectorSize(int *xpixels, int *ypixels)
{
    int x, y;
    unsigned int result = GetDetector(&x, &y);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to get detector size: %d\n", result);
        return -1; // Error
    }
    *xpixels = (int)x;
    *ypixels = (int)y;
    return 0; // Success
}

unsigned int hodr_setCoolerMode(bool mode)
{
    if (mode)
    {
        unsigned int result = CoolerON();
        if (result != DRV_SUCCESS)
        {
            fprintf(stderr, "Failed to turn on cooler: %d\n", result);
            return -1; // Error
        }
    }
    else
    {
        unsigned int result = CoolerOFF();
        if (result != DRV_SUCCESS)
        {
            fprintf(stderr, "Failed to turn off cooler: %d\n", result);
            return -1; // Error
        }
    }

    cfg.COOLER_MODE = mode ? 1 : 0; // Update configuration
    return 0;                       // Success
}

unsigned int hodr_getCurrentTemperatureFloat(float *temperature)
{
    float tempF;
    unsigned int result = GetTemperatureF(&tempF);
    *temperature = tempF;
    return result;
}
unsigned int hodr_getCurrentTemperatureInt(int *temperature)
{
    int temp;
    unsigned int result = GetTemperature(&temp);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to get current temperature: %d\n", result);
        return result; // Error
    }
    *temperature = temp;
    return result;
}
unsigned int hodr_getCurrentTemperatureAndTargetTemperature(float *temperature, float *targetTemperature)
{
    float ambientTemp, coolerVolts;
    int result = GetTemperatureStatus(temperature, targetTemperature, &ambientTemp, &coolerVolts);

    return result; // Return the result of GetTemperatureStatus
}
unsigned int hodr_getCurrentTemperatureStatus(int *temperatureStatus)
{
    int tmpInt;
    unsigned int result = GetTemperature(&tmpInt);
    *temperatureStatus = result;
    return result;
}

unsigned int hodr_getTemperatureRange(int *minTemp, int *maxTemp)
{

    unsigned int result = GetTemperatureRange(minTemp, maxTemp);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to get temperature range: %d\n", result);
    }
    return result;
}

unsigned int hodr_setTargetTemperature(int targetTemp)
{
    unsigned int result = SetTemperature(targetTemp);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to set target temperature: %d\n", result);
    }
    return result;
}

unsigned int hodr_setAcquisitionMode(int mode)
{
    unsigned int result = SetAcquisitionMode(mode);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to set acquisition mode: %d\n", result);
    }

    hodr_setKineticCycleTime(cfg.INTERVAL);    // Ensure kinetic cycle time is set after changing acquisition mode
    hodr_setNumberKinetics(cfg.SERIES_LENGTH); // Ensure number of kinetics is set after changing acquisition mode
    cfg.ACQUISITION_MODE = mode;               // Update configuration
    return result;
}

unsigned int hodr_setKineticCycleTime(float time)
{
    unsigned int result = SetKineticCycleTime(time);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to set kinetic cycle time: %d\n", result);
    }
    return result;
}

unsigned int hodr_setNumberKinetics(int number)
{
    unsigned int result = SetNumberKinetics(number);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to set number of kinetics: %d\n", result);
    }

    cfg.SERIES_LENGTH = number; // Update configuration
    return result;
}

unsigned int hodr_setReadMode(int mode)
{
    unsigned int result = SetReadMode(mode);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to set read mode: %d\n", result);
    }

    cfg.READ_MODE = mode; // Update configuration
    return result;
}

unsigned int hodr_setShutter(int type, int mode, int closingTime, int openingTime)
{
    unsigned int result = SetShutter(type, mode, closingTime, openingTime);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to set shutter: %d\n", result);
    }

    cfg.SHUTTER_TYPE = type; // Update configuration
    return result;
}

unsigned int hodr_setNumberAccumulations(int number)
{
    unsigned int result = SetNumberAccumulations(number);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to set number of accumulations: %d\n", result);
    }

    cfg.NUMBER_ACCUMULATIONS = number; // Update configuration
    return result;
}
unsigned int hodr_setExposureTime(float exposureTime)
{
    unsigned int result = SetExposureTime(exposureTime);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to set exposure time: %d\n", result);
    }
    return result;
}

unsigned int hodr_changeExposureTimeDuringSeries(float exposureTime, float *kineticCycleTime)
{
    unsigned int result = AbortAcquisition();
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to abort acquisition before changing exposure time.\n");
        return result; // Error
    }
    result = SetExposureTime(exposureTime);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to change exposure time during series: %d\n", result);
        return result; // Error
    }

    //delay for length of kinetic cycle time
    if (kineticCycleTime)
    {
        usleep((unsigned int)(*kineticCycleTime * 1000000)); // Convert seconds to microseconds
    }

    result = StartAcquisition(); // Restart acquisition after changing exposure time
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to restart acquisition after changing exposure time: %d\n", result);
        return result; // Error
    }


    cfg.INTEGRATION_TIME = exposureTime; // Update configuration
    return DRV_SUCCESS; // Success
}




unsigned int hodr_startAcquisition()
{
    unsigned int result = StartAcquisition();
    cfg.ACQ_FLAG = 0; // Reset acquisition flag
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to start acquisition: %d\n", result);
    }

    return result;
}
unsigned int hodr_startAcquisitionOnceTemperatureStabilized()
{
    // int result = hodr_setTargetTemperature(targetTemp);

    cfg.ACQ_FLAG = 1; // Set flag to indicate acquisition should start once temperature is stabilized

    return 0;
}

unsigned int hodr_isTemperatureStabilized()
{
    int temperatureStatus;

    hodr_getCurrentTemperatureStatus(&temperatureStatus);

    return (temperatureStatus == DRV_TEMP_STABILIZED ? 1 : 0); // Check if temperature is stabilized
}

unsigned int hodr_getStatus(int *status)
{
    unsigned int result = GetStatus(status);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to get status: %d\n", result);
    }
    return result;
}

unsigned int hodr_getAcquiredData(int32_t *data, size_t size)
{

    unsigned int result = GetAcquiredData(data, size);

    if (result != DRV_SUCCESS)
    {
        char acq_err_buffer[256];
        hodr_getDataAcquisitionStatusString(result, acq_err_buffer, sizeof(acq_err_buffer));
        fprintf(stderr, "Failed to get acquired data: ERROR %d - %s\n", result, acq_err_buffer);
        return result; // Error
    }
    cfg.NUMBER_ACQUISITIONS++; // Increment the number of acquisitions in the configuration

    return DRV_SUCCESS; // Success
}

unsigned int hodr_getImages(int32_t firstNewImageIndex, int32_t lastNewImageIndex, int32_t *data, size_t size, int32_t *validFirst, int32_t *validLast)
{
    unsigned int result = GetImages(firstNewImageIndex, lastNewImageIndex, data, size, validFirst, validLast);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to get images: %d\n", result);
    }
    return DRV_SUCCESS; // Success
}

unsigned int hodr_getMostRecentImage(int32_t *data, size_t size)
{
    unsigned int result = GetMostRecentImage(data, size);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to get most recent image: %d\n", result);
    }
    return result;
}

unsigned int hodr_getAcquisitionTimings(float *exposureTime, float *kineticCycleTime, float *readoutTime)
{
    unsigned int result = GetAcquisitionTimings(exposureTime, kineticCycleTime, readoutTime);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to get acquisition timings: %d\n", result);
        return result; // Error
    }
    return DRV_SUCCESS; // Success
}
unsigned int hodr_abortAcquisition()
{
    unsigned int result = AbortAcquisition();
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to abort acquisition: %d\n", result);
    }
    return result;
}

unsigned int hodr_shutDown()
{
    unsigned int result = ShutDown();
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to shut down Andor SDK: %d\n", result);
        return result; // Error
    }
    return DRV_SUCCESS; // Success
}

unsigned int hodr_getTemperatureStatusString(int status, char *buffer, size_t bufferSize)
{
    switch (status)
    {
    case DRV_TEMP_OFF:
        snprintf(buffer, bufferSize, "Temperature off");
        break;
    case DRV_TEMP_NOT_STABILIZED:
        snprintf(buffer, bufferSize, "Temperature not stabilized");
        break;
    case DRV_TEMP_STABILIZED:
        snprintf(buffer, bufferSize, "Temperature stabilized");
        break;
    case DRV_TEMP_NOT_REACHED:
        snprintf(buffer, bufferSize, "Temperature not reached");
        break;
    case DRV_TEMP_OUT_RANGE:
        snprintf(buffer, bufferSize, "Temperature out of range");
        break;
    case DRV_TEMP_NOT_SUPPORTED:
        snprintf(buffer, bufferSize, "Temperature not supported");
        break;
    case DRV_TEMP_DRIFT:
        snprintf(buffer, bufferSize, "Temperature drift detected");
        break;
    case DRV_ERROR_ACK:
        snprintf(buffer, bufferSize, "Error communicating with the camera");
        break;
    case DRV_ACQUIRING:
        snprintf(buffer, bufferSize, "Acquiring data");
        break;
    default:
        snprintf(buffer, bufferSize, "Unknown temperature status: %d", status);
    }
    return 0;
}

unsigned int hodr_getDataAcquisitionStatusString(int status, char *buffer, size_t bufferSize)
{
    switch (status)
    {
    case DRV_SUCCESS:
        snprintf(buffer, bufferSize, "Success");
        break;
    case DRV_NOT_INITIALIZED:
        snprintf(buffer, bufferSize, "Not initialized");
        break;
    case DRV_ACQUIRING:
        snprintf(buffer, bufferSize, "Acquiring data");
        break;
    case DRV_ERROR_ACK:
        snprintf(buffer, bufferSize, "Unable to communicate with instrument");
        break;
    case DRV_P1INVALID:
        snprintf(buffer, bufferSize, "Invalid pointer");
        break;
    case DRV_P2INVALID:
        snprintf(buffer, bufferSize, "Array size is incorrect");
        break;
    case DRV_NO_NEW_DATA:
        snprintf(buffer, bufferSize, "No new data available");
        break;
    default:
        snprintf(buffer, bufferSize, "Unknown status: %d", status);
    }
    return 0;
}

unsigned int hodr_getNumberAcquisitions()
{
    return cfg.NUMBER_ACQUISITIONS; // Return the number of acquisitions
}

unsigned int hodr_getAcquisitionMode()
{
    return cfg.ACQUISITION_MODE; // Return the current acquisition mode
}

unsigned int hodr_getReadMode()
{
    return cfg.READ_MODE; // Return the current read mode
}
unsigned int hodr_getShutterType()
{
    return cfg.SHUTTER_TYPE; // Return the current shutter type
}

unsigned int hodr_getNumberNewImages(int32_t *firstNewImageIndex, int32_t *lastNewImageIndex)
{
    unsigned int result = GetNumberNewImages(firstNewImageIndex, lastNewImageIndex);
    if (result != DRV_SUCCESS)
    {
        fprintf(stderr, "Failed to get number of new images: %d\n", result);
    }
    return result; // Success
}

unsigned int hodr_getOutFile(char *outFile, size_t size)
{
    if (size < sizeof(cfg.OUT_FILE))
    {
        fprintf(stderr, "Buffer size is too small for output file path\n");
        return -1; // Error
    }
    strncpy(outFile, cfg.OUT_FILE, size);
    return 0; // Success
}

unsigned int hodr_setOutFile(const char *outFile)
{
    if (strlen(outFile) >= sizeof(cfg.OUT_FILE))
    {
        fprintf(stderr, "Output file path is too long\n");
        return -1; // Error
    }
    strncpy(cfg.OUT_FILE, outFile, sizeof(cfg.OUT_FILE) - 1);
    cfg.OUT_FILE[sizeof(cfg.OUT_FILE) - 1] = '\0'; // Ensure null termination
    return 0;                                      // Success
}

unsigned int hodr_getAcqFlag()
{
    return cfg.ACQ_FLAG; // Return the acquisition flag
}
