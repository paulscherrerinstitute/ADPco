#ifndef ADPCO_H
#define ADPCO_H

#include <string>
#include <vector>

#include <epicsEvent.h>
#include <epicsMessageQueue.h>

#include <pco_device.h>
#include <sc2_camexport.h>
#ifdef __linux__
#include <pco_linux_defs.h>
#endif

#include <ADDriver.h>

#define PcoTriggerSoftwareString "PCO_TRIGGER_SOFTWARE" // asynParamInt32,   R/W
#define PcoPixelRateString       "PCO_PIX_RATE"         // asynParamInt32,   R/W
#define PcoAdcModeString         "PCO_ADC_MODE"         // asynParamInt32,   R/W
#define PcoCameraSetupString     "PCO_CAM_SETUP"        // asynParamInt32,   R/W
#define PcoReadoutModeString     "PCO_READOUT_MODE"     // asynParamInt32,   R/W
#define PcoBitAlignmentString    "PCO_BIT_ALIGNMENT"    // asynParamInt32,   R/W
#define PcoRebootCameraString    "PCO_REBOOT_CAM"       // asynParamInt32,   R/W
#define PcoAcquireModeString     "PCO_ACQ_MODE"         // asynParamInt32,   R/W
#define PcoDelayTimeString       "PCO_DELAY_TIME"       // asynParamFloat64, R/W
#define PcoFramePeriodString     "PCO_FRAME_PERIOD"     // asynParamFloat64, R/O
#define PcoFramesDroppedString   "PCO_FRAMES_DROPPED"   // asynParamInt32,   R/O

enum {
    PcoTimeBaseNanosec = 0,
    PcoTimeBaseMicrosec,
    PcoTimeBaseMillisec,
}; 

/** Main driver class inherited from areaDetectors ADGenICam class.
 * One instance of this class will control one camera.
 */
class ADPco : public ADDriver
{
public:
    ADPco(const char *portName, const char *cameraId, size_t maxMemory, int priority, int stackSize);
    virtual ~ADPco();

    // virtual methods to override from ADDriver
    virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[],
                                size_t nElements, size_t *nIn);
    virtual void report(FILE *fp, int details);

    virtual asynStatus connect(asynUser *pasynUser);

    /* These should be private but are called from C callback functions, must be public. */
    void shutdown();
    void acquisitionTask();

protected:

private:
    int PcoTriggerSoftware;
#define FIRST_PCO_PARAM PcoTriggerSoftware
    int PcoPixelRate;
    int PcoAdcMode;
    int PcoCameraSetup;
    int PcoReadoutMode;
    int PcoBitAlignment;
    int PcoRebootCamera;
    int PcoAcquireMode;
    int PcoDelayTime;
    int PcoFramePeriod;
    int PcoFramesDropped;

    /* Local methods to this class */
    asynStatus openCamera();
    asynStatus closeCamera();
    asynStatus armCamera();
    asynStatus getCameraInfo();
    asynStatus setTransferParameterAndLut();

    asynStatus startAcquisition();
    asynStatus stopAcquisition();

    void validateBin(int *bin, int maxi, bool binary);
    void validateROI(int *min, int *size, int step, int min_size, int max_size, bool symmetric, bool keep_min);
    DWORD convertSecondsToTime(double seconds, WORD *timeBase);
    double convertTimeToSeconds(DWORD time, WORD timeBase);

    std::string cameraId_;
    HANDLE cameraHandle_;

    bool exiting_;
    bool acquiring_;
    epicsEventId startEventId_;
    epicsEventId newFrameEventId_;
    
    PCO_Buflist pcoBufList_[10];
    struct {
        SHORT sBufNum;
        HANDLE hEvent;
        WORD *pData;
    } pcoBuffer_[10];

    PCO_General pcoGeneral_;
    PCO_Sensor pcoSensor_;
    PCO_Timing pcoTiming_;
    PCO_Image pcoImage_;
    WORD arraySizeX_, arraySizeY_;
    std::vector<DWORD> pcoPixelRateList_;
};

#endif
