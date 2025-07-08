#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define SHUTTER_TYP_OPEN_LOW 0
#define SHUTTER_TYP_OPEN_HIGH 1
#define SHUTTER_MODE_FULLY_AUTO 0
#define READ_MODE_FVB 0




typedef struct {
    int ACQUISITION_MODE;
    int READ_MODE;
    int COOLER_MODE; // 0 for OFF, 1 for ON
    int SHUTTER_TYPE;
    int SHUTTER_MODE; // 0 for fully auto, 1 for manual, etc.
    int SERIES_LENGTH; // Number of spectra in a kinetic series
    int NUMBER_ACQUISITIONS; // Number of acquisitions captured
    int NUMBER_ACCUMULATIONS; // Number of accumulations for the acquisition
    float INTERVAL; // Interval in seconds for acquisition
    float INTEGRATION_TIME; // Integration time in seconds

    int xpixels; // Number of horizontal pixels in the detector
    int ypixels; // Number of vertical pixels in the detector
    bool ACQ_FLAG; // Flag to indicate if acquisition should be started once temperature is stabilized
    char OUT_FILE[256]; // Output file for data
} HODR_Config_t;


unsigned int hodr_getDetectorSize(int *xpixels, int *ypixels);
unsigned int hodr_init(HODR_Config_t *config, char *andorPath, const char *outFile, bool resetConfig);
unsigned int hodr_deinit();
unsigned int hodr_setCoolerMode( bool mode);
unsigned int hodr_getCurrentTemperatureFloat(float *temperature);
unsigned int hodr_getCurrentTemperatureAndTargetTemperature(float *temperature, float *targetTemperature);
unsigned int hodr_getCurrentTemperatureInt(int *temperature);
unsigned int hodr_getCurrentTemperatureStatus(int *temperatureStatus);
unsigned int hodr_getTemperatureStatusString(int status, char *buffer, size_t bufferSize);
unsigned int hodr_getTemperatureRange(int *minTemp, int *maxTemp);
unsigned int hodr_setTargetTemperature(int targetTemp);
unsigned int hodr_setAcquisitionMode(int mode);
unsigned int hodr_setReadMode(int mode);
unsigned int hodr_setShutter(int type, int mode, int closingTime, int openingTime);
unsigned int hodr_setNumberAccumulations(int number);
unsigned int hodr_setNumberKinetics(int number);
unsigned int hodr_setExposureTime(float exposureTime);

unsigned int hodr_changeExposureTimeDuringSeries(float exposureTime, float *kineticCycleTime);
unsigned int hodr_startAcquisition();
unsigned int hodr_startAcquisitionOnceTemperatureStabilized();

unsigned int hodr_getStatus(int *status);
unsigned int hodr_getAcquiredData(int32_t *data, size_t size);
unsigned int hodr_getDataAcquisitionStatusString(int status, char *buffer, size_t bufferSize);
unsigned int hodr_getNumberNewImages(int32_t *firstNewImageIndex, int32_t *lastNewImageIndex);
unsigned int hodr_getImages(int32_t firstNewImageIndex, int32_t lastNewImageIndex, int32_t *data, size_t size, int32_t *validFirst, int32_t *validLast);
unsigned int hodr_getMostRecentImage(int32_t *data, size_t size);
unsigned int hodr_getAcquisitionTimings(float *exposureTime, float *kineticCycleTime, float *readoutTime);
unsigned int hodr_abortAcquisition();
unsigned int hodr_getAcqFlag();
unsigned int hodr_getNumberAcquisitions();

unsigned int hodr_getAcquisitionMode();
unsigned int hodr_getReadMode();
unsigned int hodr_getShutterType();
unsigned int hodr_setKineticCycleTime(float time);
unsigned int hodr_getOutFile(char *outFile, size_t size);
unsigned int hodr_setFIFOPath(const char *fifoPath);
unsigned int hodr_setOutFile(const char *outFile);
unsigned int hodr_isTemperatureStabilized();
unsigned int hodr_writeStatusToFIFO(const char *fifoPath);

