/*
 * ADPco.cpp
 *
 * This is a driver for Excelitas PCO cameras using their pco.sdk.
 *
 * Author: Xiaoqiang Wang
 *         Paul Scherrer Institute
 *
 * Created: April 11, 2025
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <algorithm>
#include <iostream>
#include <cmath>

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <cantProceed.h>
#include <epicsString.h>
#include <epicsExit.h>

#ifdef _WIN32
#include <afxwin.h>
#include <windows.h>
#endif
#ifdef __linux__
#include <pco_linux_defs.h>
#endif
#include <sc2_defs.h>
#include <sc2_sdkaddendum.h>
#include <sc2_sdkstructures.h>
#include <sc2_camexport.h>
#include <pco_device.h>
#include <pco_camexport.h>
#include <pco_err.h>

#include <epicsExport.h>
#include "ADPco.h"

#define DRIVER_VERSION      1
#define DRIVER_REVISION     0
#define DRIVER_MODIFICATION 0

static const char *driverName = "ADPco";

#define CHECK_ERROR(errorCode, API) \
    if (errorCode != PCO_NOERROR) { \
        PCO_GetErrorTextSDK(errorCode, errorText, sizeof(errorText)); \
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, \
            "%s:%s:  %s failed '%s'\n", \
            driverName, functionName, API, errorText);\
        status |= asynError; \
    }

/** Configuration function to configure one camera.
 *
 * This function need to be called once for each camera to be used by the IOC. A call to this
 * function instanciates one object from the ADPco class.
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] cameraId The camera index or serial number; <1000 is assumed to be index, >=1000 is assumed to be serial number.
 * \param[in] maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. So if max. size = 1024x768 (8bpp)
 *            and maxBuffers is, say 14. maxMemory = 1024x768x14 = 11010048 bytes (~11MB). 0=unlimited.
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
extern "C" int ADPcoConfig(const char *portName, const char *cameraId,
                             size_t maxMemory, int priority, int stackSize)
{
    new ADPco(portName, cameraId, maxMemory, priority, stackSize);
    return asynSuccess;
}

static void c_shutdown(void *drvPvt)
{
    ADPco *pPvt = (ADPco *)drvPvt;

    pPvt->shutdown();
}

static void acquisitionTaskC(void *drvPvt)
{
    ADPco *pPvt = (ADPco *) drvPvt;
    pPvt->acquisitionTask();
}


/** Constructor for the ADPco class
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] cameraId The camera index or serial number; <1000 is assumed to be index, >=1000 is assumed to be serial number.
 * \param[in] maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. So if max. size = 1024x768 (8bpp)
 *            and maxBuffers is, say 14. maxMemory = 1024x768x14 = 11010048 bytes (~11MB). 0=unlimited.
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
ADPco::ADPco(const char *portName, const char *cameraId, size_t maxMemory, int priority, int stackSize )
    : ADDriver(portName, 1, 0, 0, maxMemory,
    asynEnumMask | asynFloat64ArrayMask, asynEnumMask | asynFloat64ArrayMask,
    ASYN_CANBLOCK, 1,   /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0, autoConnect=1 */
    priority, stackSize),
    cameraId_(cameraId),
    cameraHandle_(NULL),
    exiting_(false),
    acquiring_(false)
{
    static const char *functionName = "ADPco";
    char tempString[40];
    asynStatus status;

    memset(&pcoBuffer_,0,sizeof(pcoBuffer_));

    epicsSnprintf(tempString, sizeof(tempString), "%d.%d.%d",
                  DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
    setStringParam(NDDriverVersion,tempString);

    epicsSnprintf(tempString, sizeof(tempString), "%d.%d", PCO_SDK_VERMAJOR, PCO_SDK_VERMINOR);
    setStringParam(ADSDKVersion, tempString);

    createParam(PcoTriggerSoftwareString, asynParamInt32,   &PcoTriggerSoftware);
    createParam(PcoPixelRateString,       asynParamInt32,   &PcoPixelRate);
    createParam(PcoAdcModeString,         asynParamInt32,   &PcoAdcMode);
    createParam(PcoCameraSetupString,     asynParamInt32,   &PcoCameraSetup);
    createParam(PcoReadoutModeString,     asynParamInt32,   &PcoReadoutMode);
    createParam(PcoBitAlignmentString,    asynParamInt32,   &PcoBitAlignment);
    createParam(PcoRebootCameraString,    asynParamInt32,   &PcoRebootCamera);
    createParam(PcoAcquireModeString,     asynParamInt32,   &PcoAcquireMode);
    createParam(PcoDelayTimeString,       asynParamFloat64, &PcoDelayTime);
    createParam(PcoFramePeriodString,     asynParamFloat64, &PcoFramePeriod);
    createParam(PcoFramesDroppedString,   asynParamInt32,   &PcoFramesDropped);

    /* Set initial values of some parameters */
    setStringParam(ADManufacturer, "Excelitas PCO");
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setIntegerParam(ADMinX, 0);
    setIntegerParam(ADMinY, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");

    PCO_InitializeLib();

    status = openCamera();
    if (status == asynSuccess) {
        getCameraInfo();
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s:  camera connection failed (%d)\n",
            driverName, functionName, status);
        // Call report() to get a list of available cameras
        report(stdout, 1);

        this->disconnect(pasynUserSelf);
        setIntegerParam(ADStatus, ADStatusDisconnected);
        setStringParam(ADStatusMessage, "Camera disconnected");    
    }

    startEventId_ = epicsEventCreate(epicsEventEmpty);
    newFrameEventId_ = epicsEventCreate(epicsEventEmpty);
    /* Create the acquisition thread */
    epicsThreadCreate("ADPcoAcquisitionTask",
        epicsThreadPriorityMedium, epicsThreadGetStackSize(
        epicsThreadStackMedium),
        (EPICSTHREADFUNC) acquisitionTaskC, this);

    // shutdown on exit
    epicsAtExit(c_shutdown, this);

    return;
}

ADPco::~ADPco()
{
}

/** Called by epicsAtExit.
 *  Here we close the camera connection and terminate the PCO system.
*/
void ADPco::shutdown(void)
{
    fprintf(stderr, "%s:%s: shutting down\n", driverName, __FUNCTION__);
    lock();
    exiting_ = true;
    if (cameraHandle_) {
        setIntegerParam(ADAcquire, 0);
        stopAcquisition();
        closeCamera();    
    }
    unlock();
    PCO_CleanupLib();
}

void ADPco::acquisitionTask()
{
    int status = asynSuccess;
    int adStatus;
    int acquire;
    int arrayCallbacks;
    int imageMode, imageCounter;
    int numImages, numImagesCounter;
    int framesDropped;
    epicsTimeStamp startTime;
    std::string statusMessage;
    int errorCode;
    char errorText[100] = {0};
    size_t dims[2];
    NDArray *pImage;
    const char *functionName = "acquisitionTask";

    this->lock();
    while (1)
    {
        getIntegerParam(ADAcquire, &acquire);
        getIntegerParam(ADStatus, &adStatus);
        /* If we are not acquiring or encountered a problem then wait for a semaphore
         * that is given when acquisition is started */
        if (!acquire)
        {
            /* Only set the status message if we didn't encounter a problem last time,
             * so we don't overwrite the error mesage */
            if(adStatus == (int)ADStatusIdle)
            {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s:%s: Waiting for the acquire command\n",
                        driverName, functionName);
                setStringParam(ADStatusMessage, "Waiting for the acquire command");
                callParamCallbacks();
            }
            /* Release the lock while we wait for the start event then lock again */
            this->unlock();
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: Waiting for acquire to start\n",
                    driverName, functionName);
            epicsEventWait(this->startEventId_);
            this->lock();
            /* We are acquiring. */
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: We are acquiring\n",
                    driverName, functionName);

            dims[0] = arraySizeX_;
            dims[1] = arraySizeY_;
            getIntegerParam(ADAcquire, &acquire);
            setIntegerParam(ADNumImagesCounter, 0);
            setIntegerParam(ADStatus, ADStatusAcquire);
            setIntegerParam(PcoFramesDropped, 0);
            setIntegerParam(NDArraySizeX, arraySizeX_);
            setIntegerParam(NDArraySizeY, arraySizeY_);
            setIntegerParam(NDArraySize, arraySizeX_ * arraySizeY_ * sizeof(WORD));
            setStringParam(ADStatusMessage, "Acquiring....");
            callParamCallbacks();
        }
        epicsTimeGetCurrent(&startTime);

        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(PcoFramesDropped, &framesDropped);

        this->unlock();

        #ifdef __linux__
        SHORT bufNum;
        errorCode = PCO_WaitforNextBufferNum(cameraHandle_, &bufNum, 2000);
        #else
        SHORT bufNum;
        errorCode = PCO_WaitforBuffer(cameraHandle_, sizeof(pcoBufList_)/sizeof(PCO_Buflist), pcoBufList_, 2000);
        for (size_t i=0; i<sizeof(pcoBufList_)/sizeof(PCO_Buflist); i++) {
            if (pcoBufList_[i].dwStatusDrv == PCO_NOERROR) {
                bufNum = pcoBufList_[i].sBufNr;
                break;
            }
        }
        #endif
        if (errorCode == PCO_NOERROR) {
            DWORD statusDll, statusDrv;
            HANDLE eventHandle;
            errorCode = PCO_GetBufferStatus(cameraHandle_, bufNum, &statusDll, &statusDrv);
            if (errorCode == PCO_NOERROR && statusDrv == PCO_NOERROR) {
                WORD *pData;
                errorCode = PCO_GetBuffer(cameraHandle_, bufNum, &pData, &eventHandle);
                if (errorCode == PCO_NOERROR) {
                    pImage = this->pNDArrayPool->alloc(2, dims, NDUInt16, 0, NULL);
                    memcpy(pImage->pData, pData, arraySizeX_ * arraySizeY_ * sizeof(WORD));
                    numImagesCounter++;
                    imageCounter++;
                }
                errorCode = PCO_AddBufferEx(cameraHandle_, 0, 0, bufNum,  arraySizeX_, arraySizeY_, sizeof(WORD));
                CHECK_ERROR(errorCode, "PCO_AddBufferEx");
            }
        } else if ((errorCode & PCO_ERROR_TIMEOUT) != PCO_ERROR_TIMEOUT &&
                   (errorCode & PCO_ERROR_DRIVER_BUFFER_CANCELLED) != PCO_ERROR_DRIVER_BUFFER_CANCELLED) {
            framesDropped++;
            CHECK_ERROR(errorCode, "PCO_WaitforNextBufferNum");
        }

        lock();
        setIntegerParam(NDArrayCounter, imageCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);
        setIntegerParam(PcoFramesDropped, framesDropped);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);

        if (pImage) {
            updateTimeStamp(&pImage->epicsTS);
            pImage->timeStamp = pImage->epicsTS.secPastEpoch + pImage->epicsTS.nsec/1e9;

            // Get any attributes that have been defined for this driver        
            getAttributes(pImage->pAttributeList);
            
            if (arrayCallbacks)
                this->doCallbacksGenericPointer(pImage, NDArrayData, 0);

            pImage->release();
            pImage = NULL;
        }

        // See if acquisition is done if we are in single or multiple mode
        if ((imageMode == ADImageSingle && numImagesCounter > 0) || ((imageMode == ADImageMultiple) && (numImagesCounter >= numImages))) {
            acquire = 0;
            setIntegerParam(ADAcquire, acquire);
            stopAcquisition();
        }
        callParamCallbacks();
    }
}


asynStatus ADPco::connect(asynUser *pasynUser)
{
    static const char *functionName = "connect";
    static bool first = true;

    if (first) {
        first = false;
        return asynError;
    }

    if (!this->deviceIsReachable) {
        asynStatus status = openCamera();
        if (status != asynSuccess) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s:  camera connection failed (%d)\n",
                driverName, functionName, status);
            return status;
        }
        this->deviceIsReachable = true;   
        getCameraInfo();
    }

    return ADDriver::connect(pasynUser);
}

asynStatus ADPco::openCamera()
{
    static const char *functionName = "openCamera";
    int status = asynSuccess;
    int errorCode = PCO_NOERROR;
    char errorText[100] = {0};

    // Scan the interfaces first. Later opening camera is more likely to succeed.
    #ifdef __linux__
    WORD count = 0;
    PCO_Device devices[10] = {0};
    errorCode = PCO_ScanCameras(PCO_INTERFACE_ALL, &count, devices, sizeof(devices));
    CHECK_ERROR(errorCode, "PCO_ScanCameras");
    if (status) return (asynStatus)status;
    #endif

    // If cameraId_ is a number < 1000, use it as the index
    if (cameraId_.size()<4 && std::all_of(cameraId_.begin(), cameraId_.end(), isdigit)) {
        errorCode = PCO_OpenCamera(&cameraHandle_, std::stoi(cameraId_));
        CHECK_ERROR(errorCode, "PCO_OpenCamera");
    }
    #ifdef __linux__
    else {
        bool found = false;
        for (size_t i=0; i<count; i++) {
            if (cameraId_ == std::to_string(devices[i].SerialNumber)) {
                found = true;
                errorCode = PCO_OpenCameraDevice(&cameraHandle_, (WORD)devices[i].id);
                CHECK_ERROR(errorCode, "PCO_OpenCameraDevice");
                break;
            }
        }
        if (!found) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: camera with S/N '%s' not found\n",
                driverName, functionName, cameraId_.c_str());
            return asynError;
        }
    }
    #endif
    if (status) return (asynStatus)status;

    errorCode = PCO_SetRecordingState(cameraHandle_, 0);
    CHECK_ERROR(errorCode, "PCO_SetRecordingState");
    if (status) return (asynStatus)status;

    errorCode = PCO_ResetSettingsToDefault(cameraHandle_);
    CHECK_ERROR(errorCode, "PCO_ResetSettingsToDefault");
    if (status) return (asynStatus)status;

    return asynSuccess;
}


asynStatus ADPco::closeCamera()
{
    static const char *functionName = "closeCamera";
    int status = asynSuccess;
    int errorCode = PCO_NOERROR;
    char errorText[100] = {0};

    if (cameraHandle_ == NULL)
        return asynSuccess;

    errorCode = PCO_CloseCamera(cameraHandle_);
    CHECK_ERROR(errorCode, "PCO_CloseCamera");
    if (status) return (asynStatus)status;

    cameraHandle_ = NULL;
    this->deviceIsReachable = false;
    this->disconnect(pasynUserSelf);
    setIntegerParam(ADStatus, ADStatusDisconnected);
    setStringParam(ADStatusMessage, "Camera disconnected");

    return asynSuccess;
}

asynStatus ADPco::armCamera()
{
    static const char *functionName = "armCamera";
    int status = asynSuccess;
    DWORD dwWarning, dwError, dwStatus;
    int errorCode = PCO_NOERROR;
    char errorText[100] = {0};

    errorCode = PCO_ArmCamera(cameraHandle_);
    CHECK_ERROR(errorCode, "PCO_ArmCamera");
    if (status) return (asynStatus)status;

    errorCode = PCO_GetCameraHealthStatus(cameraHandle_, &dwWarning, &dwError, &dwStatus);
    CHECK_ERROR(errorCode, "PCO_GetCameraHealthStatus");
    if (status) return (asynStatus)status;

    status |= setTransferParameterAndLut();

    status |= getCameraInfo();

    return (asynStatus)status;
}


asynStatus ADPco::startAcquisition()
{
    static const char *functionName = "startAcquisition";
    int status = asynSuccess;
    int errorCode;
    char errorText[100] = {0};

    /* Allocate buffers */
    for (size_t i=0; i<10; i++) {
        pcoBuffer_[i].sBufNum = -1;
        pcoBuffer_[i].hEvent = NULL;
        pcoBuffer_[i].pData = NULL;

        errorCode = PCO_AllocateBuffer(cameraHandle_, 
            &pcoBuffer_[i].sBufNum, 
            arraySizeX_ * arraySizeY_ * sizeof(WORD),
            &pcoBuffer_[i].pData,
            &pcoBuffer_[i].hEvent
        );
        CHECK_ERROR(errorCode, "PCO_AllocateBuffer");
        if (status) return (asynStatus)status;

        pcoBufList_[i].sBufNr = pcoBuffer_[i].sBufNum;
    }

    /* For non-edge cameras, switch on recording state before buffers given */
    if (pcoGeneral_.strCamType.wCamType != CAMERATYPE_PCO_EDGE &&
        pcoGeneral_.strCamType.wCamType != CAMERATYPE_PCO_EDGE_42 &&
        pcoGeneral_.strCamType.wCamType != CAMERATYPE_PCO_EDGE_GL &&
        pcoGeneral_.strCamType.wCamType != CAMERATYPE_PCO_EDGE_HS &&
        pcoGeneral_.strCamType.wCamType != CAMERATYPE_PCO_EDGE_MT) {
        errorCode = PCO_SetRecordingState(cameraHandle_, 1);
        CHECK_ERROR(errorCode, "PCO_SetRecordingState");
        if (status) return (asynStatus)status;        
    }

    for (size_t i=0; i<10; i++) {
        errorCode = PCO_AddBufferEx(cameraHandle_, 0, 0, pcoBuffer_[i].sBufNum, arraySizeX_, arraySizeY_, sizeof(WORD));
        CHECK_ERROR(errorCode, "PCO_AddBufferEx");
        if (status) return (asynStatus)status;
    }

    /* For edge cameras, switch on recording state after buffers given */
    if (pcoGeneral_.strCamType.wCamType == CAMERATYPE_PCO_EDGE ||
        pcoGeneral_.strCamType.wCamType == CAMERATYPE_PCO_EDGE_42 ||
        pcoGeneral_.strCamType.wCamType == CAMERATYPE_PCO_EDGE_GL ||
        pcoGeneral_.strCamType.wCamType == CAMERATYPE_PCO_EDGE_HS ||
        pcoGeneral_.strCamType.wCamType == CAMERATYPE_PCO_EDGE_MT) {
        errorCode = PCO_SetRecordingState(cameraHandle_, 1);
        CHECK_ERROR(errorCode, "PCO_SetRecordingState");
        if (status) return (asynStatus)status;
    }

    return asynSuccess;
}


asynStatus ADPco::stopAcquisition()
{
    static const char *functionName = "stopAcquisition";
    int status = asynSuccess;
    int errorCode;
    char errorText[100] = {0};

    errorCode = PCO_CancelImages(cameraHandle_);
    CHECK_ERROR(errorCode, "PCO_CancelImages");
    if (status) return (asynStatus)status;

    errorCode = PCO_SetRecordingState(cameraHandle_, 0);
    CHECK_ERROR(errorCode, "PCO_SetRecordingState");
    if (status) return (asynStatus)status;

    setIntegerParam(ADStatus, ADStatusIdle);
    setStringParam(ADStatusMessage, "Idle");

    for (size_t i=0; i<10; i++) {
        if (pcoBuffer_[i].pData == NULL)
            continue;

        errorCode = PCO_FreeBuffer(cameraHandle_, pcoBuffer_[i].sBufNum);
        CHECK_ERROR(errorCode, "PCO_FreeBuffer");
        pcoBuffer_[i] = {0};
    }

    return (asynStatus)status;
}

/** From PCO SDK documentation:
  * For Global Shutter setup the data format cannot be changed. The available data format is:
  *     PCO_CL_DATAFORMAT_5x12 | SCCMOS_FORMAT_TOP_CENTER_BOTTOM_CENTER
  * 
  * pco.edge 5.5 Rolling Shutter and GlobalReset mode:
  * 
  * | Sensor Pixelrate | horizontal Resolution | PCO_CL_Dataformat       | Lookup Table |
  * +------------------+-----------------------+-------------------------+--------------+
  * | 95 MHz           | all                   | PCO_CL_DATAFORMAT_5x16  | 0            |
  * +------------------+-----------------------+-------------------------+--------------+
  * |                  | below or equal 1920   | PCO_CL_DATAFORMAT_5x16  | 0            |
  * | 286 MHz          +-----------------------+-------------------------+--------------+
  * |                  | above 1920            | PCO_CL_DATAFORMAT_5x12L | 0x1612       |
  * |                  |                       | PCO_CL_DATAFORMAT_5x12R | 0x1612       |
  * +------------------+-----------------------+-------------------------+--------------+
  *
  * pco.edge 4.2 Rolling Shutter and GlobalReset mode:
  * | Sensor Pixelrate | horizontal Resolution | PCO_CL_Dataformat       | Lookup Table |
  * +------------------+-----------------------+-------------------------+--------------+
  * | 95 MHz           | all                   | PCO_CL_DATAFORMAT_5x16  | 0            |
  * +------------------+-----------------------+-------------------------+--------------+
  * | 286 MHz          | all                   | PCO_CL_DATAFORMAT_5x16  | 0            |
  * +------------------+-----------------------+-------------------------+--------------+
  */
asynStatus ADPco::setTransferParameterAndLut()
{
    const char *functionName = "setTransferParameterAndLut";
    int status = asynSuccess;
    int errorCode = PCO_NOERROR;
    char errorText[100] = {0};

    /* Only for cameralink interfaces */
    if (pcoGeneral_.strCamType.wInterfaceType != INTERFACE_CAMERALINK &&
        pcoGeneral_.strCamType.wInterfaceType != INTERFACE_CAMERALINKHS) {
        return asynSuccess;
    }

    PCO_SC2_CL_TRANSFER_PARAM transferParam;
    errorCode = PCO_GetTransferParameter(cameraHandle_, &transferParam, sizeof(transferParam));
    CHECK_ERROR(errorCode, "PCO_GetTransferParameter");
    if (status) return (asynStatus)status;

    int cameraSetup;
    getIntegerParam(PcoCameraSetup, &cameraSetup);

    int readoutMode;
    getIntegerParam(PcoReadoutMode, &readoutMode);
    readoutMode = (readoutMode << 8);

    DWORD dataformat = PCO_CL_DATAFORMAT_1x16;
    WORD lutId = 0, lutParam = 0;

    if (cameraSetup == PCO_EDGE_SETUP_GLOBAL_SHUTTER) {
        dataformat = PCO_CL_DATAFORMAT_5x12 | SCCMOS_FORMAT_TOP_CENTER_BOTTOM_CENTER;
    } else {   
        dataformat = PCO_CL_DATAFORMAT_5x16 | readoutMode;
        if (pcoGeneral_.strCamType.wCamSubType == CAMERASUBTYPE_PCO_EDGE_55) {
            if (pcoSensor_.dwPixelRate == 286000000 && arraySizeX_ > 1920) {
                dataformat = PCO_CL_DATAFORMAT_5x12L | readoutMode;
                lutId = 0x1612;
            }
        }
    }

    transferParam.DataFormat = dataformat;
    errorCode = PCO_SetTransferParameter(cameraHandle_, &transferParam, sizeof(transferParam));
    CHECK_ERROR(errorCode, "PCO_SetTransferParameter");
    if (status) return (asynStatus)status;

    /* Set Lut but ignore "not suppported" error. */
    errorCode = PCO_SetActiveLookupTable(cameraHandle_, &lutId, &lutParam);
    if ((errorCode & PCO_ERROR_FIRMWARE_NOT_SUPPORTED) != PCO_ERROR_FIRMWARE_NOT_SUPPORTED) {
        CHECK_ERROR(errorCode, "PCO_SetActiveLookupTable");
    }
    if (status) return (asynStatus)status;

    return (asynStatus)status;
}

asynStatus ADPco::getCameraInfo()
{
    static const char *functionName = "getCameraInfo";
    int status = asynSuccess;
    int errorCode = PCO_NOERROR;
    char errorText[100] = {0};
    char cameraName[40] = {0};
    std::string temp;

    pcoGeneral_ = {0};
    pcoGeneral_.wSize = sizeof(PCO_General);
    pcoGeneral_.strCamType.wSize = sizeof(PCO_CameraType);
    errorCode = PCO_GetGeneral(cameraHandle_, &pcoGeneral_);
    CHECK_ERROR(errorCode, "PCO_GetGeneral");
    if (status) return (asynStatus)status;

    temp = std::to_string(pcoGeneral_.strCamType.dwSerialNumber);
    setStringParam(ADSerialNumber, temp.c_str());
    temp = std::to_string(pcoGeneral_.strCamType.dwFWVersion);
    setStringParam(ADFirmwareVersion, temp.c_str());

    setDoubleParam(ADTemperatureActual, pcoGeneral_.sCamTemperature);

    if (PCO_GetCameraName(cameraHandle_, cameraName, sizeof(cameraName)) == PCO_NOERROR) {
        setStringParam(ADModel, cameraName);
    }

    pcoSensor_ = {0};
    pcoSensor_.wSize = sizeof(PCO_Sensor);
    pcoSensor_.strDescription.wSize = sizeof(PCO_Description);
    pcoSensor_.strDescription2.wSize = sizeof(PCO_Description2);
    pcoSensor_.strDescription3.wSize = sizeof(PCO_Description3);
    pcoSensor_.strSignalDesc.wSize = sizeof(PCO_Signal_Description);
    pcoSensor_.strDescriptionIntensified.wSize = sizeof(PCO_Description_Intensified);

    errorCode = PCO_GetSensorStruct(cameraHandle_, &pcoSensor_);
    CHECK_ERROR(errorCode, "PCO_GetSensorStruct");
    if (status) return (asynStatus)status;

    pcoTiming_ = {0};
    pcoTiming_.wSize = sizeof(PCO_Timing);
    for (size_t i=0; i<sizeof(pcoTiming_.strSignal)/sizeof(PCO_Signal); i++) {
        pcoTiming_.strSignal[i].wSize = sizeof(PCO_Signal);
    }
    errorCode = PCO_GetTimingStruct(cameraHandle_, &pcoTiming_);
    CHECK_ERROR(errorCode, "PCO_GetTimingStruct");
    if (status) return (asynStatus)status;

    pcoImage_ = {0};
    pcoImage_.wSize = sizeof(PCO_Image);
    pcoImage_.strColorSet.wSize = sizeof(PCO_Image_ColorSet);
    for (size_t i=0; i<sizeof(pcoImage_.strSegment)/sizeof(PCO_Segment); i++) {
        pcoImage_.strSegment[i].wSize = sizeof(PCO_Segment);
    }
    errorCode = PCO_GetImageStruct(cameraHandle_, &pcoImage_);
    CHECK_ERROR(errorCode, "PCO_GetImageStruct");
    if (status) return (asynStatus)status;

    setIntegerParam(ADMaxSizeX, pcoSensor_.strDescription.wMaxHorzResStdDESC);
    setIntegerParam(ADMaxSizeY, pcoSensor_.strDescription.wMaxVertResStdDESC);

    setIntegerParam(ADBinX, pcoSensor_.wBinHorz);
    setIntegerParam(ADBinY, pcoSensor_.wBinVert);

    setIntegerParam(ADMinX, pcoSensor_.wRoiX0 - 1);
    setIntegerParam(ADMinY, pcoSensor_.wRoiY0 - 1);

    arraySizeX_ = pcoSensor_.wRoiX1 - pcoSensor_.wRoiX0 + 1;
    arraySizeY_ = pcoSensor_.wRoiY1 - pcoSensor_.wRoiY0 + 1;
    setIntegerParam(ADSizeX, arraySizeX_);
    setIntegerParam(ADSizeY, arraySizeY_);

    DWORD exposureTime, delayTime;
    WORD exposureTimeBase, delayTimeBase;
    errorCode = PCO_GetDelayExposureTime(cameraHandle_, &delayTime, &exposureTime, &delayTimeBase, &exposureTimeBase);
    CHECK_ERROR(errorCode, "PCO_GetDelayExposureTime");
    if (status == asynSuccess) {
        setDoubleParam(ADAcquireTime, convertTimeToSeconds(exposureTime, exposureTimeBase));
        setDoubleParam(PcoDelayTime, convertTimeToSeconds(delayTime, delayTimeBase));
    }

    pcoPixelRateList_.clear();
    size_t num = sizeof(pcoSensor_.strDescription.dwPixelRateDESC) / sizeof(DWORD);
    for (size_t i=0; i<num; i++) {
        if (pcoSensor_.strDescription.dwPixelRateDESC[i] == 0) break;
        pcoPixelRateList_.push_back(pcoSensor_.strDescription.dwPixelRateDESC[i]);
    }

    for (size_t i=0; i<pcoPixelRateList_.size(); i++) {
        if (pcoSensor_.dwPixelRate == pcoPixelRateList_[i]) {
            setIntegerParam(PcoPixelRate, (int)i);
            break;
        }
    }

    setIntegerParam(PcoAdcMode, pcoSensor_.wADCOperation);

    setIntegerParam(PcoBitAlignment, pcoImage_.wBitAlignment);

    WORD mode;
    errorCode = PCO_GetShutterMode(cameraHandle_, &mode);
    CHECK_ERROR(errorCode, "PCO_GetShutterMode");
    if (status == asynSuccess) {
        setIntegerParam(PcoCameraSetup, mode);
    }

    setIntegerParam(ADTriggerMode, pcoTiming_.wTriggerMode);

    DWORD period_s, period_ns;
    errorCode = PCO_GetCOCRuntime(cameraHandle_, &period_s, &period_ns);
    CHECK_ERROR(errorCode, "PCO_GetCOCRuntime");
    if (status == asynSuccess) {
        setDoubleParam(PcoFramePeriod, period_s + period_ns / 1e9);
    }

    WORD acquireMode;
    errorCode = PCO_GetAcquireMode(cameraHandle_, &acquireMode);
    CHECK_ERROR(errorCode, "PCO_GetAcquireMode");
    if (status == asynSuccess) {
        setIntegerParam(PcoAcquireMode, acquireMode);
    }
 
    WORD interfaceType=2, format=0, reserved1=0, reserved2=0;
    errorCode = PCO_GetInterfaceOutputFormat(cameraHandle_, &interfaceType, &format, &reserved1, &reserved2);
    CHECK_ERROR(errorCode, "PCO_GetInterfaceOutputFormat");
    if (status == asynSuccess) {
        setIntegerParam(PcoReadoutMode, (format >> 8));
    }

    return asynSuccess;
}

void ADPco::validateBin(int *bin, int maxi, bool binary)
{
    if (*bin < 1) *bin = 1;

    /* binning has to be 2^n */
    if (binary) {
        bool is_2_n = *bin & (*bin-1);
        if (!is_2_n)
            *bin = 1;
    }

    if (*bin > maxi)
        *bin = maxi;
}

/** Hardware ROI can be very strictive, e.g.
  * 1. the start and size must be a multiple of step.
  * 2. the region must be centered in the sensor.
  * \param[inout] min The minimum value of the ROI.
  * \param[inout] size The size of the ROI.
  * \param[in] stepSize The step size.
  * \param[in] minSize The minimum size of the ROI.
  * \param[in] maxSize The maximum size of the ROI.
  * \param[in] symmetric If true, the ROI must be symmetric.
  * \param[in] keepMin If true, the min value has priority. */
void ADPco::validateROI(int *min, int *size, int stepSize, int minSize, int maxSize, bool symmetric, bool keepMin)
{
    if (keepMin) {
        *min = std::min<int>( std::max<int>(*min, 0), symmetric ? (maxSize / 2 - minSize / 2) : (maxSize - minSize) );

        int numSteps = *min / stepSize;
        *min = numSteps * stepSize;

        if (symmetric) {
            *size = maxSize - *min * 2;
        } else {
            *size = std::min<int>(std::max<int>(*size, minSize), maxSize - *min);
            *size = (*size / stepSize) * stepSize;
        }
    } else {
        if (symmetric) {
            *size = std::min<int>(std::max<int>(*size, stepSize * 2), maxSize);
            *min = (maxSize - *size) / 2 / stepSize * stepSize;
            *size = maxSize - *min * 2;
        } else {
            *size = std::min<int>(std::max<int>(*size, minSize), maxSize - *min);
            *size = (*size / stepSize) * stepSize;
        }
    }
}

DWORD ADPco::convertSecondsToTime(double seconds, WORD *timeBase)
{
    if (seconds < 0.0004) {
        *timeBase = PcoTimeBaseNanosec;
        return (DWORD)(seconds * 1e9);
    } else if (seconds < 4) {
        *timeBase = PcoTimeBaseMicrosec;
        return (DWORD)(seconds * 1e6);
    } else {
        *timeBase = PcoTimeBaseMillisec;
        return (DWORD)(seconds * 1e3);
    }
}

double ADPco::convertTimeToSeconds(DWORD time, WORD timeBase)
{
    switch (timeBase) {
        case PcoTimeBaseNanosec:
            return (double)time / 1e9;
        case PcoTimeBaseMicrosec:
            return (double)time / 1e6;
        case PcoTimeBaseMillisec:
            return (double)time / 1e3;
    }

    /* It should never reach here. */
    return (double)time;
}

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADBinX, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus ADPco::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int status = asynSuccess;
    int errorCode = 0;
    char errorText[100] = {0};
    static const char *functionName = "writeInt32";
 
    /* Set the parameter and readback in the parameter library.
     * This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= setIntegerParam(function, value);

    if (function == ADAcquire) {
        if (value) {
            epicsEventSignal(startEventId_);
            status = startAcquisition();
        } else {
            status = stopAcquisition();
        }
     } else if (function == ADTriggerMode) {
        errorCode = PCO_SetTriggerMode(cameraHandle_, value);
        CHECK_ERROR(errorCode, "PCO_SetTriggerMode");
        status |= armCamera();
     } else if (function == ADMinX ||
                function == ADMinY ||
                function == ADSizeX ||
                function == ADSizeY ||
                function == ADBinX ||
                function == ADBinY) {
        int minX, minY, sizeX, sizeY, binX, binY, maxSizeX, maxSizeY;
        int effectiveSizeX, effectiveSizeY;
        int symmetricROIX = 0, symmetricROIY = 0;
        getIntegerParam(ADMinX, &minX);
        getIntegerParam(ADMinY, &minY);
        getIntegerParam(ADSizeX, &sizeX);
        getIntegerParam(ADSizeY, &sizeY);
        getIntegerParam(ADBinX, &binX);
        getIntegerParam(ADBinY, &binY);
        getIntegerParam(ADMaxSizeX, &maxSizeX);
        getIntegerParam(ADMaxSizeY, &maxSizeY);

        if ((pcoSensor_.strDescription.dwGeneralCapsDESC1 & GENERALCAPS1_ROI_HORZ_SYMM_TO_VERT_AXIS) ||
            pcoSensor_.wADCOperation == 2 ||
            (pcoGeneral_.strCamType.wCamType & CAMERATYPE_PCO_DIMAX_STD) == CAMERATYPE_PCO_DIMAX_STD)
            symmetricROIX = 1;
        if ((pcoSensor_.strDescription.dwGeneralCapsDESC1 & GENERALCAPS1_ROI_VERT_SYMM_TO_HORZ_AXIS) ||
            (pcoGeneral_.strCamType.wCamType & CAMERATYPE_PCO_EDGE) == CAMERATYPE_PCO_EDGE)
            symmetricROIY = 1;

        validateBin(&binX,
            pcoSensor_.strDescription.wMaxBinHorzDESC,
            pcoSensor_.strDescription.wBinHorzSteppingDESC==0
        );
        validateBin(&binY,
            pcoSensor_.strDescription.wMaxBinVertDESC,
            pcoSensor_.strDescription.wBinVertSteppingDESC==0
        );

        effectiveSizeX = maxSizeX / binX;
        effectiveSizeY = maxSizeY / binY;

        validateROI(&minX, &sizeX,
            pcoSensor_.strDescription.wRoiHorStepsDESC,
            pcoSensor_.strDescription.wMinSizeHorzDESC,
            effectiveSizeX,
            symmetricROIX,
            function == ADMinX
        );

        validateROI(&minY, &sizeY,
            pcoSensor_.strDescription.wRoiVertStepsDESC,
            pcoSensor_.strDescription.wMinSizeVertDESC,
            effectiveSizeY,
            symmetricROIY,
            function == ADMinY
        );

        errorCode = PCO_SetBinning(cameraHandle_, binX, binY);
        CHECK_ERROR(errorCode, "PCO_SetBinning"); 

        errorCode = PCO_SetROI(cameraHandle_, minX + 1, minY + 1, minX + sizeX, minY + sizeY);
        CHECK_ERROR(errorCode, "PCO_SetROI");

        errorCode = PCO_SetImageParameters(cameraHandle_, sizeX, sizeY,
            IMAGEPARAMETERS_READ_WHILE_RECORDING, NULL, 0);
        CHECK_ERROR(errorCode, "PCO_SetImageParameters");
    
        status |= armCamera();
    } else if (function == PcoTriggerSoftware) {
        WORD triggered = 0;
        errorCode = PCO_ForceTrigger(cameraHandle_, &triggered);
        CHECK_ERROR(errorCode, "PCO_ForceTrigger");
        if (!triggered) {
            status |= asynError;
        }
        setIntegerParam(PcoTriggerSoftware, 0);
    } else if (function == PcoAdcMode) {
        if (value > 0) {
            errorCode = PCO_SetADCOperation(cameraHandle_, value);
            CHECK_ERROR(errorCode, "PCO_SetADCOperation");
            status |= armCamera();    
        }
    } else if (function == PcoPixelRate) {
        if (value >= 0 && value < (int)pcoPixelRateList_.size()) {
            errorCode = PCO_SetPixelRate(cameraHandle_, pcoPixelRateList_[value]);
            CHECK_ERROR(errorCode, "PCO_SetPixelRate");
            status |= armCamera();
        }
    } else if (function == PcoCameraSetup) {
        errorCode = PCO_SetShutterMode(cameraHandle_, value);
        CHECK_ERROR(errorCode, "PCO_SetShutterMode");
        if (status == asynSuccess) {
            errorCode = PCO_RebootCamera(cameraHandle_);
            CHECK_ERROR(errorCode, "PCO_RebootCamera");    
            if (status == asynSuccess) {
                status |= closeCamera();
            }
        }
    } else if (function == PcoReadoutMode) {
        errorCode = PCO_SetInterfaceOutputFormat(cameraHandle_, 2, value << 8, 0, 0);
        CHECK_ERROR(errorCode, "PCO_SetInterfaceOutputFormat");
        status |= armCamera();
    } else if (function == PcoBitAlignment) {
        errorCode = PCO_SetBitAlignment(cameraHandle_, value);
        CHECK_ERROR(errorCode, "PCO_SetBitAlignment");
        status |= armCamera();
    } else if (function == PcoAcquireMode) {
        errorCode = PCO_SetAcquireMode(cameraHandle_, value);
        CHECK_ERROR(errorCode, "PCO_SetAcquireMode");
        status |= armCamera();
    } else if (function == PcoRebootCamera) {
        setIntegerParam(PcoRebootCamera, 0);

        errorCode = PCO_RebootCamera(cameraHandle_);
        CHECK_ERROR(errorCode, "PCO_RebootCamera");
        status |= closeCamera();
    } else if (function < FIRST_PCO_PARAM) {
        status = ADDriver::writeInt32(pasynUser, value);
    }

    callParamCallbacks();

     if (status)
         asynPrint(pasynUser, ASYN_TRACE_ERROR,
               "%s:%s: error, status=%d function=%d, value=%d\n",
               driverName, functionName, status, function, value);
     else
         asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
               "%s:%s: function=%d, value=%d\n",
               driverName, functionName, function, value);

    return (asynStatus)status;
}


/** Called when asyn clients call pasynFloat64->write().
  * For all  parameters it  sets the value in the parameter library and calls any registered callbacks.
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus ADPco::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    int status = asynSuccess;
    int errorCode = 0;
    char errorText[100] = {0};
    const char* functionName = "writeFloat64";

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setDoubleParam(function, value);

    if (function == ADAcquireTime ||
        function == PcoDelayTime) {
        double exposure, delay;
        getDoubleParam(ADAcquireTime, &exposure);
        getDoubleParam(PcoDelayTime, &delay);

        exposure = std::min<double>( std::max<double>(exposure, pcoSensor_.strDescription.dwMinExposureDESC * 1e-9),  pcoSensor_.strDescription.dwMaxExposureDESC * 1e-3);
        delay = std::min<double>( std::max<double>(delay, pcoSensor_.strDescription.dwMinDelayDESC * 1e-9),  pcoSensor_.strDescription.dwMaxDelayDESC * 1e-3);
        
        DWORD exposureTime, delayTime;
        WORD exposureTimeBase, delayTimeBase;

        exposureTime = convertSecondsToTime(exposure, &exposureTimeBase);
        delayTime = convertSecondsToTime(delay, &delayTimeBase);

        errorCode = PCO_SetDelayExposureTime(cameraHandle_, delayTime,  exposureTime, delayTimeBase, exposureTimeBase);
        CHECK_ERROR(errorCode, "PCO_SetDelayExposureTime");

        /* PCO SDK documentation:
           When the Recording State of the camera is [run], camera timing is changed immediately (best possible),
           else new settings will be valid after a call of PCO_ArmCamera. */
        WORD recordingState;
        PCO_GetRecordingState(cameraHandle_, &recordingState);
        if (recordingState == 0)
            status |= armCamera();

        DWORD period_s, period_ns;
        errorCode = PCO_GetCOCRuntime(cameraHandle_, &period_s, &period_ns);
        CHECK_ERROR(errorCode, "PCO_GetCOCRuntime");
        if (status == asynSuccess) {
            setDoubleParam(PcoFramePeriod, period_s + period_ns / 1e9);    
        }
    } else {
        if (function < FIRST_PCO_PARAM)
            status = ADDriver::writeFloat64(pasynUser, value);
    }

    callParamCallbacks();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%g\n",
              driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%g\n",
              driverName, functionName, function, value);
    return((asynStatus)status);   
}


asynStatus ADPco::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[],
                               size_t nElements, size_t *nIn)
{
    int function = pasynUser->reason;

    if (function == PcoPixelRate) {
        *nIn = 0;
        for (size_t i=0; i<pcoPixelRateList_.size() && i<nElements; i++) {
            if (strings[*nIn]) free(strings[*nIn]);
            std::string option = std::to_string(pcoPixelRateList_[i]/1000000) + " MHz";
            strings[*nIn] = epicsStrDup(option.c_str());
            values[*nIn] = (int)i;
            severities[*nIn] = 0;
            (*nIn)++;
        }
        return asynSuccess;
    } if (function == NDColorMode) {
        if (strings[0]) free(strings[0]);
        strings[0] = epicsStrDup("Mono");
        values[0] = NDColorModeMono;
        *nIn = 1;
        return asynSuccess;
    } else if (function == NDDataType) {
        if (strings[0]) free(strings[0]);
        strings[0] = epicsStrDup("UInt16");
        values[0] = NDUInt16;
        *nIn = 1;
        return asynSuccess;
    } else if (function == PcoCameraSetup) {
        *nIn = 0;
        if (strings[*nIn]) free(strings[*nIn]);
        strings[*nIn] = epicsStrDup("Rolling Shutter");
        values[*nIn] = PCO_EDGE_SETUP_ROLLING_SHUTTER;
        severities[*nIn] = 0;
        (*nIn)++;
        if ((pcoSensor_.strDescription.dwGeneralCapsDESC1 & GENERALCAPS1_NO_GLOBAL_SHUTTER) == 0) {
            if (strings[*nIn]) free(strings[*nIn]);
            strings[*nIn] = epicsStrDup("Global Shutter");
            values[*nIn] = PCO_EDGE_SETUP_GLOBAL_SHUTTER;
            severities[*nIn] = 0;
            (*nIn)++;
        }
        if (pcoSensor_.strDescription.dwGeneralCapsDESC1 & GENERALCAPS1_GLOBAL_RESET_MODE) {
            if (strings[*nIn]) free(strings[*nIn]);
            strings[*nIn] = epicsStrDup("Global Reset");
            values[*nIn] = PCO_EDGE_SETUP_GLOBAL_RESET;
            severities[*nIn] = 0;
            (*nIn)++;
        }
        return asynSuccess;
    } else if (function == PcoAdcMode) {
        /* Somehow this is reported as 0 by a PCO edge 5.5 CLHS.
           So only change enum choices if the number is valid. */
        *nIn = 0;
        if (pcoSensor_.strDescription.wNumADCsDESC == 0) {
            return asynError;
        }
        if (pcoSensor_.strDescription.wNumADCsDESC >= 1) {
            if (strings[*nIn]) free(strings[*nIn]);
            strings[*nIn] = epicsStrDup("Single");
            values[*nIn] = 1;
            severities[*nIn] = 0;
            (*nIn)++;    
        }
        if (pcoSensor_.strDescription.wNumADCsDESC >= 2) {
            if (strings[*nIn]) free(strings[*nIn]);
            strings[*nIn] = epicsStrDup("Dual");
            values[*nIn] = 2;
            severities[*nIn] = 0;
            (*nIn)++;
        }
        return asynSuccess;
    }

    return ADDriver::readEnum(pasynUser, strings, values, severities, nElements, nIn);
}


void ADPco::report(FILE *fp, int details)
{
    if (details > 1) {
        #ifdef __linux__
        WORD count = 0;
        PCO_Device devices[10] = {0};
        PCO_ScanCameras(PCO_INTERFACE_ALL, &count, devices, sizeof(devices));
        fprintf(fp, "\nNumber of cameras detected: %d\n", count);
        for (int i=0; i<count; i++) {
            fprintf(fp, "Camera %d: %s\n", i, devices[i].CameraName);
            fprintf(fp, "  Camera ID: %d\n", devices[i].id);
            fprintf(fp, "  Serial number: %d\n", devices[i].SerialNumber);
            fprintf(fp, "  Camera type: %d\n", devices[i].CameraType);
            fprintf(fp, "  Camera interface: %s\n", devices[i].PCO_InterfaceName);
            fprintf(fp, "  Camera extended info: %" PRIu64 "\n", devices[i].ExtendedInfo);
        }
        #endif
    }
    ADDriver::report(fp, details);
    return;
}


static const iocshArg configArg0 = {"Port name", iocshArgString};
static const iocshArg configArg1 = {"cameraId", iocshArgString};
static const iocshArg configArg2 = {"maxMemory", iocshArgInt};
static const iocshArg configArg3 = {"priority", iocshArgInt};
static const iocshArg configArg4 = {"stackSize", iocshArgInt};
static const iocshArg * const configArgs[] = {&configArg0,
                                              &configArg1,
                                              &configArg2,
                                              &configArg3,
                                              &configArg4};
static const iocshFuncDef configADPco = {"ADPcoConfig", 5, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
    ADPcoConfig(args[0].sval, args[1].sval, args[2].ival,
                  args[3].ival, args[4].ival);
}


static void ADPcoRegister(void)
{
    iocshRegister(&configADPco, configCallFunc);
}

extern "C" {
epicsExportRegistrar(ADPcoRegister);
}
