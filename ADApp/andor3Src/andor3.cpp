/*
 * andor3.cpp
 *
 * This is a driver for Andor SDK3 camera (Neo/Zyla)
 *
 * Author: Phillip Sorensen
 *         Cornell University
 *
 * Created: October 8, 2012
 *
 * NOTES:
 *
 * 1. Feature "CameraAcquiring" is a boolen with FALSE = 0 and TRUE = 1.  This
 *    is mapped to parameter ADStatus and relies on the fact the current
 *    definition of the mbbi/mbbo are 0 is "Idle", and 1 is "Acquiring".  No
 *    other states of ADStatus are currently used.  If default mapping is
 *    changed the mbbi/mbbo record will need to be overriden like the 
 *    trigger and image mode records. 
 *
 * 2. Memory for buffers passed to the Camera API are allocated with the
 *    posix_memalign funtion in linux.  For other platforms this may need
 *    to be changed.
 */

#include <stdio.h>
#include <string.h>
#include <wchar.h>

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsString.h>
#include <epicsExport.h>
#include <epicsExit.h>

#include <atcore.h>

#include "ADDriver.h"

#define MAX_FEATURE_NAME_LEN 64

static const char *driverName = "andor3";

static int AtInitialized = 0;

/* feature types */
typedef enum {
    ATint,
    ATfloat,
    ATbool,
    ATenum,
    ATstring
} Andor3FeatureType;

typedef struct {
  class andor3*     camera;
  Andor3FeatureType type;
  int               paramIndex;
  AT_BOOL           isImplemented;
  AT_WC             featureName[MAX_FEATURE_NAME_LEN];
  bool              exists;
} featureInfo;


class andor3 : public ADDriver {
public:
    andor3(const char *portName, int cameraId, int maxBuffers, 
           size_t maxMemory, int priority, int stackSize, int maxFrames);

    /* override ADDriver methods */ 
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], 
                                size_t nElements, size_t *nIn);

    /* "private methods" that need to be called from C */
    void shutdown();
    void imageTask();
    void tempTask();

    int getFeature(const AT_WC *feature, Andor3FeatureType type,
                   int paramIndex);
protected:
    int Andor3FrameRate;
    #define FIRST_ANDOR3_PARAM Andor3FrameRate
    int Andor3PixelEncoding;
    int Andor3FullAOIControl;
    int Andor3Binning;
    int Andor3ShutterMode;
    int Andor3SoftwareTrigger;
    int Andor3SensorCooling;
    int Andor3TempControl;
    int Andor3TempStatus;
    int Andor3SerialNumber;
    int Andor3FirmwareVersion;
    int Andor3SoftwareVersion;
    int Andor3ControllerID;
    int Andor3Overlap;
    int Andor3ReadoutRate;
    int Andor3ReadoutTime;
    int Andor3PreAmpGain;
    int Andor3NoiseFilter;
    int Andor3FanSpeed;
    #define LAST_ANDOR3_PARAM Andor3FanSpeed
private:
    int setFeature(const AT_WC *feature, Andor3FeatureType type,
                   int paramIndex);
    int registerFeature(const AT_WC *feature, Andor3FeatureType type,
                        int paramIndex);
    int getFeature(const AT_WC *feature, Andor3FeatureType type,
                   int paramIndex, AT_H handle);
    int updateAOI(int set);
    int allocateBuffers();
    int freeBuffers();
    int connectCamera();
    int disconnectCamera();
    
    featureInfo *featureInfo_;
    AT_H    handle_;
    int     id_;
    int     maxFrames_;
    AT_U8 **drvBuffers_;    
    AT_64   imageSize_;
    int     exiting_;
    epicsEventId startEvent_;
};
#define NUM_ANDOR3_PARAMS (&LAST_ANDOR3_PARAM - &FIRST_ANDOR3_PARAM + 1)

/* Andor3 driver specific parameters */
#define Andor3FrameRateString        "A3_FRAME_RATE"        /* asynFloat64  rw */
#define Andor3PixelEncodingString    "A3_PIXEL_ENCODING"    /* asynInt32    rw */
#define Andor3FullAOIControlString   "A3_FULL_AOI_CONTROL"  /* asynInt32    ro */
#define Andor3BinningString          "A3_BINNING"           /* asynInt32    rw */
#define Andor3ShutterModeString      "A3_SHUTTER_MODE"      /* asynInt32    rw */
#define Andor3SoftwareTriggerString  "A3_SOFTWARE_TRIGGER"  /* asynInt32    wo */
#define Andor3SensorCoolingString    "A3_SENSOR_COOLING"    /* asynInt32    rw */
#define Andor3TempControlString      "A3_TEMP_CONTROL"      /* asynInt32    rw */
#define Andor3TempStatusString       "A3_TEMP_STATUS"       /* asynInt32    ro */
#define Andor3SerialNumberString     "A3_SERIAL_NUMBER"     /* asynOctet    ro */
#define Andor3FirmwareVersionString  "A3_FIRMWARE_VERSION"  /* asynOctet    ro */
#define Andor3SoftwareVersionString  "A3_SOFTWARE_VERSION"  /* asynOctet    ro */
#define Andor3ControllerIDString     "A3_CONTROLLER_ID"     /* asynOctet    ro */
#define Andor3OverlapString          "A3_OVERLAP"           /* asynInt32    rw */
#define Andor3ReadoutRateString      "A3_READOUT_RATE"      /* asynInt32    rw */
#define Andor3ReadoutTimeString      "A3_READOUT_TIME"      /* asynFloat64  rw */
#define Andor3PreAmpGainString       "A3_PREAMP_GAIN"       /* asynInt32    rw */
#define Andor3NoiseFilterString      "A3_NOISE_FILTER"      /* asynInt32    rw */
#define Andor3FanSpeedString         "A3_FAN_SPEED"         /* asynInt32    rw */

/* Andor3 specific enumerations - sync with mbbi records */
typedef enum  {
    ATFixed = 0,
    ATContinuous = 1
} ATCycleMode_t;  /* "CycleMode"  $(P)$(R)ImageMode */

typedef enum {
    ATMono12 = 0,
    ATMono12Packed = 1,
    ATMono16 = 2
} ATPixelEncoding_t;  /* "PixelEncoding"  $(P)$(R)PixelEncoding */


static void c_shutdown(void *arg)
{
  andor3 *p = (andor3 *)arg;
  p->shutdown();
}

static void c_imagetask(void *arg)
{
  andor3 *p = (andor3 *)arg;
  p->imageTask();
}

static void c_temptask(void *arg)
{
  andor3 *p = (andor3 *)arg;
  p->tempTask();
}

static int AT_EXP_CONV c_getfeature(AT_H handle, const AT_WC *feature, void *context)
{
  featureInfo *info = (featureInfo *)context;
  return info->camera->getFeature(feature, info->type, info->paramIndex);
}


void andor3::shutdown(void)
{
    exiting_ = 1;
    if(handle_) {
        disconnectCamera();
    }
    AtInitialized--;
    if(AtInitialized == 0) {
        AT_FinaliseLibrary();
    }
}

void andor3::imageTask()
{
    epicsTimeStamp imageStamp;
    int status;
    AT_U8  *image;
    int size;
    int acquire;
    int mode;
    int total;
    int number;
    int count;
    int callback;

    static const char *functionName = "imageTask";

    lock();

    while(!exiting_) {

        getIntegerParam(ADAcquire, &acquire);
        if(!acquire) {
            AT_Flush(handle_);

            unlock();
            epicsEventWait(startEvent_);
            lock();
            
            AT_Flush(handle_);
            for(int x=0; x<maxFrames_; x++) {
                if(drvBuffers_[x]) {
                    status = AT_QueueBuffer(handle_, drvBuffers_[x],
                                            imageSize_);
                    if(status) {
                        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                            "%s:%s: AT_QueueBuffer error: %d\n", 
                            driverName, functionName, status);
                    }
                }
            }
            AT_Command(handle_, L"AcquisitionStart");
        }
        
        unlock();
        status = AT_WaitBuffer(handle_, &image, &size, AT_INFINITE);
        lock();
        if(status != AT_SUCCESS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: AT_WaitBuffer, error=%d\n", 
                driverName, functionName, status);
            continue;
        }
        epicsTimeGetCurrent(&imageStamp);

        getIntegerParam(ADNumImagesCounter, &number);
        number++;
        setIntegerParam(ADNumImagesCounter, number);
        getIntegerParam(NDArrayCounter, &count);
        count++;
        setIntegerParam(NDArrayCounter, count);
        callParamCallbacks();

        getIntegerParam(NDArrayCallbacks, &callback);
        if(callback) {
            NDArray *pImage;
            int      dims[2];

            getIntegerParam(NDArraySizeX, &dims[0]);
            getIntegerParam(NDArraySizeY, &dims[1]);

            pImage = pNDArrayPool->alloc(2, dims, NDUInt16, 0, NULL);
            if(pImage) {
                int encoding;
                AT_64 stride;
                int pixelSize = 2;

                pImage->uniqueId = count;
                pImage->timeStamp = 631152000 + imageStamp.secPastEpoch +
                    (imageStamp.nsec / 1.0e9);

                getIntegerParam(Andor3PixelEncoding, &encoding);
                AT_GetInt(handle_, L"AOIStride", &stride);
                if(encoding == ATMono12 || encoding == ATMono16) {
                    AT_U8 *p;

                    p = (AT_U8 *)pImage->pData;
                    for(int x = 0; x < size; x += stride) {
                        memcpy(p, image+x, dims[0]*pixelSize);
                        p += dims[0]*pixelSize;
                    }
                } else if(encoding == ATMono12Packed) {
                    AT_U8 *enc = image;
                    unsigned short *dec = (unsigned short*)pImage->pData;

                    for(int x = 0; x < size; x += stride) {
                        enc = image + x;
                        for (int j = 0; j < dims[0]/pixelSize; j++) {
                            *dec     = (*enc << 4) + (*(enc+1) & 0xf);
                            *(dec+1) = (*(enc+2)<<4) + ((*(enc+1) >> 4) & 0xf);
                            enc += 3;
                            dec += pixelSize;
                        }
                    }
                }

                getAttributes(pImage->pAttributeList);

                unlock();
                doCallbacksGenericPointer(pImage, NDArrayData, 0);
                lock();

                pImage->release();
            }
        }
        getIntegerParam(ADImageMode, &mode);
        getIntegerParam(ADNumImages, &total);
        if(mode == ATFixed && number == total) {
            setShutter(0);
            AT_Command(handle_, L"AcquisitionStop");
            setIntegerParam(ADAcquire, 0);
        }
        callParamCallbacks();

        status = AT_QueueBuffer(handle_, image, size);
        if(status) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: AT_QueueBuffer 2: error=%d\n", 
                driverName, functionName, status);
        }
    }
}

void andor3::tempTask(void)
{
    int status;
    static const char *functionName = "tempTask";

    while(!exiting_) {
        status  = getFeature(L"SensorTemperature", ATfloat, ADTemperatureActual);
        status |= getFeature(L"SensorCooling",     ATbool,  Andor3SensorCooling);
        status |= getFeature(L"TemperatureStatus", ATenum,  Andor3TempStatus);

        if(status) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: temperature read error = %d\n", 
                driverName, functionName, status);
        }
        epicsThreadSleep(1.0);
    }
}

int andor3::getFeature(const AT_WC *feature, Andor3FeatureType type,
                       int paramIndex)
{
    return getFeature(feature, type, paramIndex, handle_);
}

int andor3::getFeature(const AT_WC *feature, Andor3FeatureType type,
                       int paramIndex, AT_H handle)
{
    static const char* functionName = "getFeature";

    int     status = 0;
    char    featureName[MAX_FEATURE_NAME_LEN];
    AT_64   i_value;
    double  d_value;
    AT_BOOL b_value;
    int     index;
    int     length;
    AT_WC  *wide;
    char   *str;

    wcstombs(featureName, feature, MAX_FEATURE_NAME_LEN);

    /* get feature value to paramIndex */
    switch(type) {
    case ATint:
        status = AT_GetInt(handle, feature, &i_value);
        if(status == AT_SUCCESS) {
            status = setIntegerParam(paramIndex, (int)i_value);
            if(status) {
                status = -1;
            }
        }
        break;
    case ATfloat:
        status = AT_GetFloat(handle, feature, &d_value);
        if(status == AT_SUCCESS) {
            status = setDoubleParam(paramIndex, d_value);
            if(status) {
                status = -1;
            }
        }
        break;
    case ATbool:
        status = AT_GetBool(handle, feature, &b_value);
        if(status == AT_SUCCESS) {
            status = setIntegerParam(paramIndex, (int)b_value);
            if(status) {
                status = -1;
            }
        }
        break;
    case ATenum:
        status = AT_GetEnumIndex(handle, feature, &index);
        if(status == AT_SUCCESS) {
            status = setIntegerParam(paramIndex, index);
            if(status) {
                status = -1;
            }
        }
        break;
    case ATstring:
        status = AT_GetStringMaxLength(handle, feature, &length);
        if(status == AT_SUCCESS) {
            length++;

            wide = new AT_WC[length];
            str  = new char[length];

            status = AT_GetString(handle, feature, wide, length);
            if(status == AT_SUCCESS) {
                wcstombs(str, wide, length);
                status = setStringParam(paramIndex, str);
                if(status) {
                    status = -1;
                }
            }
            delete wide;
            delete str;
        }
    }

    /* translate features to ADBase parameters */
    if(status == AT_SUCCESS) {
        if(!strcmp(featureName, "FrameRate")) {
            paramIndex = ADAcquirePeriod;
            status = setDoubleParam(paramIndex, d_value ? 1.0/d_value : 0);
            if(status) {
                status = -1;
            }
        }
        if(!strncmp(featureName, "AOI", 3)) {
            updateAOI(0);
        }       
    }

    /* error messages */
    if(status == -1) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unable to set parameter index %d\n",
            driverName, functionName, paramIndex);
        status = AT_ERR_NOTWRITABLE;
    } else if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unable to get feature %s (%d)\n",
            driverName, functionName, featureName, status);   
    }

    callParamCallbacks();
    return status;
}

int andor3::setFeature(const AT_WC *feature, Andor3FeatureType type,
                       int paramIndex)
{
    static const char* functionName = "setFeature";

    char   featureName[MAX_FEATURE_NAME_LEN];
    int    status=0;
    int    i_value;
    AT_64  i_min;
    AT_64  i_max;
    double d_value;
    double d_min;
    double d_max;


    wcstombs(featureName, feature, MAX_FEATURE_NAME_LEN);

    /* get feature value to paramIndex */
    switch(type) {
    case ATint:
        status = getIntegerParam(paramIndex, &i_value);
        if(status) {
            status = -1;
        } else {
            status  = AT_GetIntMax(handle_, feature, &i_max);
            status |= AT_GetIntMin(handle_, feature, &i_min);
            if(status == AT_SUCCESS) {
                if(i_value < i_min) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s setting %s to minimum value %d (was %d)\n",
                        driverName, functionName, featureName, 
                              i_min, i_value);
                    i_value = (int)i_min;
                } else if(i_value > i_max) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s setting %s to max value %d (was %d)\n",
                        driverName, functionName, featureName, 
                              i_max, i_value);
                    i_value = (int)i_max;
                }
                status = AT_SetInt(handle_, feature, (AT_64)i_value);
            }
        }
        break;
    case ATfloat:
        status = getDoubleParam(paramIndex, &d_value);
        if(status) {
            status = -1;
        } else {
            status  = AT_GetFloatMax(handle_, feature, &d_max);
            status |= AT_GetFloatMin(handle_, feature, &d_min);
            if(status == AT_SUCCESS) {
                if(d_value < d_min) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s setting %s to minimum value %f (was %f)\n",
                          driverName, functionName, featureName, 
                          d_min, d_value);
                    d_value = d_min;
                } else if(d_value > d_max) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s setting %s to max value %f (was %f)\n",
                        driverName, functionName, featureName, 
                        d_max, d_value);
                    d_value = d_max;
                }
                status = AT_SetFloat(handle_, feature, d_value);
            }
        }
        break;
    case ATbool:
        status = getIntegerParam(paramIndex, &i_value);
        if(status) {
            status = -1;
        } else {
            status = AT_SetBool(handle_, feature, 
                                i_value ? AT_TRUE : AT_FALSE);
        }
        break;
    case ATenum:
        status = getIntegerParam(paramIndex, &i_value);
        if(status) {
            status = -1;
        } else {
            status = AT_SetEnumIndex(handle_, feature, i_value);
        }
        break;
    case ATstring:
        status = 2;
        break;
    }

    /* error messages */
    if(status == -1) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unable to get parameter index %d\n",
            driverName, functionName, paramIndex);
        status = AT_ERR_NOTWRITABLE;
    } else if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unable to set feature %s (%d)\n",
            driverName, functionName, featureName, status);   
    }

    /* read back current value if set failed */
    if(status != AT_SUCCESS) {
        getFeature(feature, type, paramIndex);
    }
    return status;
}

int andor3::registerFeature(const AT_WC *feature, Andor3FeatureType type,
                            int paramIndex)
{
    static const char *functionName = "registerFeature";
    int  status = -1;
    char featureName[MAX_FEATURE_NAME_LEN];

    wcstombs(featureName, feature, MAX_FEATURE_NAME_LEN);

    featureInfo *info = &featureInfo_[paramIndex];
    info->camera = this;
    info->type = type;
    info->paramIndex = paramIndex;
    wcscpy(info->featureName, feature);
    info->exists = true;
    
    status  = AT_IsImplemented(handle_, feature, &info->isImplemented);
    if (!info->isImplemented) return 0;
    status = AT_RegisterFeatureCallback(handle_, feature, 
                                         c_getfeature, info);
    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unable to register feature %s (%d)\n",
            driverName, functionName, featureName, status);   
    }
    return status;
}

/* if set  true  -> update camera from parameters 
           false -> update parameters from camera */
int andor3::updateAOI(int set)
{
    int   status=0;
    AT_64 at64Value;
    int minX, sizeX, binX;
    int minY, sizeY, binY;
    AT_64 sizeI;
    int binning;
    int binValues[] = {1, 2, 3, 4, 8};
    static const char *functionName = "updateAOI";


    if(set) {
        status |= getIntegerParam(ADSizeX, &sizeX);
        status |= getIntegerParam(ADSizeY, &sizeY);
        status |= getIntegerParam(Andor3Binning, &binning);
        status |= getIntegerParam(ADMinX, &minX);
        status |= getIntegerParam(ADMinY, &minY);
    } else {
        status |= AT_GetInt(handle_, L"AOIWidth",  &at64Value); sizeX = at64Value;
        status |= AT_GetInt(handle_, L"AOILeft",   &at64Value); minX  = at64Value;
        status |= AT_GetInt(handle_, L"AOIHeight", &at64Value); sizeY = at64Value;
        status |= AT_GetInt(handle_, L"AOITop",    &at64Value); minY  = at64Value;
        status |= AT_GetEnumIndex(handle_, L"AOIBinning", &binning);
    }
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error getting values, set=%d, error=%d\n",
            driverName, functionName, set, status);   
        return status;
    }
    binX = binValues[binning];
    setIntegerParam(ADBinX, binX);
    binY = binValues[binning];
    setIntegerParam(ADBinY, binY);
    if(set) {
        status |= AT_SetEnumIndex(handle_, L"AOIBinning", binning);
        status |= AT_SetInt(handle_, L"AOIWidth",  sizeX/binX);
        status |= AT_SetInt(handle_, L"AOILeft",   minX);
        status |= AT_SetInt(handle_, L"AOIHeight", sizeY/binY);
        status |= AT_SetInt(handle_, L"AOITop",    minY);
    } else {
        status |= setIntegerParam(ADSizeX, sizeX*binX);
        status |= setIntegerParam(ADMinX,  minX);
        status |= setIntegerParam(ADSizeY, sizeY*binY);
        status |= setIntegerParam(ADMinY,  minY);
        status |= setIntegerParam(Andor3Binning, binning);
    }   

    /* set NDArray parameters */
    status |= setIntegerParam(NDArraySizeX, sizeX/binX);
    status |= setIntegerParam(NDArraySizeY, sizeY/binY);
    status |= setIntegerParam(NDArraySize,  sizeX/binX * sizeY/binY * 2);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error setting values, set=%d, error=%d\n",
            driverName, functionName, set, status);   
        return status;
    }
    /* reallocate image buffers if size changed */
    status = AT_GetInt(handle_, L"ImageSizeBytes", &sizeI);
    if(status == AT_SUCCESS && sizeI != imageSize_) {
        allocateBuffers();
    }
    return status;
}

int andor3::allocateBuffers(void)
{
    int   status;
    AT_64 size;

    freeBuffers();

    status = AT_GetInt(handle_, L"ImageSizeBytes", &size);
    if(status != AT_SUCCESS) {
        size = 11059200; /* 16 bit full size */
        status = AT_SUCCESS;
    }
    imageSize_ = size;

    drvBuffers_ = (AT_U8 **)calloc(maxFrames_, sizeof(AT_U8 *));
    if(drvBuffers_) {
        for(int x = 0; x < maxFrames_; x++) {
            #ifdef _WIN32
               drvBuffers_[x] = (AT_U8*)_aligned_malloc(size, 8);
            #else
                /* allocate 8 byte aligned buffer */
                if(!posix_memalign((void **)&drvBuffers_[x], 8, size)) {
                    //status |= AT_QueueBuffer(handle_,
                    //                         drvBuffers[x], (int)size);
                } else {   
                    drvBuffers_[x] = NULL;
                }
            #endif
        }
    }

    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:allocateBuffers: Failed to allocate and queue buffers\n",
            driverName);
    }   
    return status;
}

int andor3::freeBuffers()
{
    int status;

    status = AT_Flush(handle_);
    if(drvBuffers_) {
        for(int x = 0; x < maxFrames_; x++) {
            if(drvBuffers_[x]) {
                #ifdef _WIN32
                    _aligned_free(drvBuffers_[x]);
                #else
                    free(drvBuffers_[x]);
                #endif
                drvBuffers_[x] = NULL;
            }
        }
        free(drvBuffers_);
        drvBuffers_ = NULL;
    }
    return status;
}

    
int andor3::connectCamera(void)
{
    static const char *functionName = "connectCamera";

    int status = AT_SUCCESS;


    /* disconnect any connected camera first */
    disconnectCamera();

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: connecting camera %d\n",
        driverName, functionName, id_);

    /* open handle to camera */
    status = AT_Open(id_, &handle_);
    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unable to open camera %d\n",
            driverName, functionName, id_);
        return status;
    }

    allocateBuffers();
    return status;
}

int andor3::disconnectCamera(void) 
{
    static const char *functionName = "disconnectCamera";

    int status;
    int acquiring;


    if(!handle_) {
        return AT_SUCCESS;
    }

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: disconnecting camera %d\n",
        driverName, functionName, id_);

    status = AT_GetBool(handle_, L"CameraAcquiring", &acquiring);
    if(status == AT_SUCCESS && acquiring) {
        status |= AT_Command(handle_, L"Acquisition Stop");
    }

    status |= freeBuffers();
    status |= AT_Close(handle_);

    if(status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error closing camera %d\n",
            driverName, functionName, id_);
    }

    handle_ = 0;
    return status;
}

asynStatus andor3::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], 
                            size_t nElements, size_t *nIn)
{
    int index = pasynUser->reason;
    int i;
    int len;
    int enumCount;
    mbstate_t mbs = {0};
    AT_WC enumStringWC[MAX_FEATURE_NAME_LEN];
    const AT_WC *pWC;
    char enumString[MAX_FEATURE_NAME_LEN];
    AT_BOOL isImplemented;
    int status;
    static const char *functionName = "readEnum";

    *nIn = 0;
    // Get the featureInfo for this parameter, set if it exists, is implemented, and is ATenum
    featureInfo *info = &featureInfo_[index];    
    if (!info->exists) return asynError;
    if (!info->isImplemented) {
        if (strings[0]) free(strings[0]);
        if (strings[1]) free(strings[1]);
        if (info->type == ATbool) {
            strings[0] = epicsStrDup("N.A. 0");
            strings[1] = epicsStrDup("N.A. 1");
            *nIn = 2;
        } else {
            strings[0] = epicsStrDup("N.A.");
            *nIn = 1;
        }
        return asynSuccess;
    }
    if (info->type != ATenum) return asynError;
    
    status = AT_GetEnumCount(handle_, info->featureName, &enumCount);
    for (i=0; ((i<enumCount) && (i<(int)nElements)); i++) {
        status |= AT_IsEnumIndexImplemented(handle_, info->featureName, i, &isImplemented);
        if (!isImplemented) continue;
        if (strings[*nIn]) free(strings[*nIn]);
        status |= AT_GetEnumStringByIndex(handle_, info->featureName, i, 
                                          enumStringWC, MAX_FEATURE_NAME_LEN-1);
        pWC = enumStringWC;
        len = wcsrtombs(enumString, &pWC, sizeof(enumString)-1, &mbs);  
        strings[*nIn] = epicsStrDup(enumString);
        values[*nIn] = i;
        severities[*nIn] = 0;
        (*nIn)++;
    }
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: error calling AT enum functions, status=%d\n",
            driverName, functionName, status);
    }
    return asynSuccess;
}



asynStatus andor3::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    static const char *functionName = "writeInt32";
    const char *paramName;

    int index = pasynUser->reason;
    int status;

    getParamName(index, &paramName);

    /* set "locked" parameters */
    if(index == NDDataType) {
        value = NDUInt16;
    } else if(index == NDColorMode) {
        value = NDColorModeMono;
    }

    /* set parameter from value */
    status = setIntegerParam(index, value);
    if(status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: failed to write parameter %s = %f\n",
            driverName, functionName, paramName, value);
        return (asynStatus)status;
    }

    if(index == ADAcquire) {
        if(value) {
            setIntegerParam(ADNumImagesCounter, 0);
            setShutter(1);
            epicsEventSignal(startEvent_);
        } else {
            setShutter(0);
            status = AT_Command(handle_, L"AcquisitionStop");
        }
    }
    else if(index == ADImageMode) {
        status = setFeature(L"CycleMode", ATenum, ADImageMode);        
    }
    else if(index == ADTriggerMode) {
        status = setFeature(L"TriggerMode", ATenum, ADTriggerMode);
    }
    else if(index == ADNumExposures) {
        status = setFeature(L"AccumulateCount", ATint, ADNumExposures);
    }
    else if(index == ADNumImages) {
        status = setFeature(L"FrameCount", ATint, ADNumImages);
    }
    else if 
       ((index == Andor3Binning) ||
        (index == ADMinX)  ||
        (index == ADSizeX) ||
        (index == ADMinY)  ||
        (index == ADSizeY)) {
        status |= updateAOI(1);
    }
    else if(index == ADReadStatus) {
        status  = getFeature(L"CameraAcquiring", ATbool, ADStatus);
        status |= updateAOI(0);
    }
    else if(index == Andor3PixelEncoding) {
        status = setFeature(L"PixelEncoding", ATenum, Andor3PixelEncoding);
        if(status == AT_SUCCESS) {
            status = allocateBuffers();
        }
    }
    else if(index == Andor3SensorCooling) {
        status = setFeature(L"SensorCooling", ATbool, Andor3SensorCooling);
    }
    else if(index == Andor3ShutterMode) {
        status = setFeature(L"ElectronicShutteringMode", ATenum, 
                            Andor3ShutterMode);
    }
    else if(index == Andor3SoftwareTrigger) {
        if(value) {
            status = AT_Command(handle_, L"SoftwareTrigger");
            setIntegerParam(Andor3SoftwareTrigger, 0);
        }
    }
    else if(index == Andor3TempControl) {
        status = setFeature(L"TemperatureControl", ATenum, Andor3TempControl);
    }
    else if(index == Andor3Overlap) {
        status = setFeature(L"Overlap", ATbool, Andor3Overlap);
    }
    else if(index == Andor3FanSpeed) {
        status = setFeature(L"FanSpeed", ATenum, Andor3FanSpeed);
    }
    else if(index == Andor3ReadoutRate) {
        status = setFeature(L"PixelReadoutRate", ATenum, Andor3ReadoutRate);
    }
    else if(index == Andor3PreAmpGain) {
        status = setFeature(L"SimplePreAmpGainControl", ATenum, Andor3PreAmpGain);
    }
    else if(index == Andor3NoiseFilter) {
        status = setFeature(L"SpuriousNoiseFilter", ATbool, Andor3NoiseFilter);
    }
    else {
        if(index < FIRST_ANDOR3_PARAM) {
            status = ADDriver::writeInt32(pasynUser, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if(status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: error, status=%d param=%s(%d) value=%d\n",
            driverName, functionName, status, paramName, index, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: param=%s(%d) value=%d\n",
            driverName, functionName, paramName, index, value);
    }
    return (asynStatus)status;
}

asynStatus andor3::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    static const char *functionName = "writeFloat64";
    const char *paramName;

    int index = pasynUser->reason;
    int status = asynSuccess;

    getParamName(index, &paramName);

    /* set parameter from value */
    status = setDoubleParam(index, value);
    if(status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: failed to write parameter %s = %f\n",
            driverName, functionName, paramName, value);
        return (asynStatus)status;
    }

    if(index == ADTemperature) {
        status = setFeature(L"TargetSensorTemperature", ATfloat, ADTemperature);
    }
    else if(index == ADAcquireTime) {
        status = setFeature(L"ExposureTime", ATfloat, ADAcquireTime);
    }
    else if(index == ADAcquirePeriod) {
        status = setDoubleParam(Andor3FrameRate, value ? 1.0/value : 0);
        if(!status) {
            status = setFeature(L"FrameRate", ATfloat, Andor3FrameRate);
        }
    }
    else if(index == Andor3FrameRate) {
        status = setFeature(L"FrameRate", ATfloat, Andor3FrameRate);
    }
    else {    
        if(index < FIRST_ANDOR3_PARAM) {
            status = ADDriver::writeFloat64(pasynUser, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if(status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: error, status=%d param=%s(%d) value=%f\n",
            driverName, functionName, status, paramName, index, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: param=%s(%d) value=%f\n",
            driverName, functionName, paramName, index, value);
    }
    return (asynStatus)status;
}



extern "C" int andor3Config(const char *portName, int cameraId, int maxBuffers,
                            size_t maxMemory, int priority, int stackSize,
                            int maxFrames)
{
    new andor3(portName, cameraId, maxBuffers, maxMemory, priority, stackSize,
               maxFrames);
    return(asynSuccess);
}

/** Constructor for Andor3 driver; most parameters are simply passed to
  * ADDriver::ADDriver.
  *
  * After calling the base class constructor this method creates a thread to
  * collect the images from the detector and sets reasonable default values for
  * the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  *
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] cameraId The id number of the Andor camera (see listdevices
  *            example for number).
  * \param[in] maxBuffers The maximum number of NDArray buffers that the
  *            NDArrayPool for this driver is allowed to allocate. Set this to
  *            -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for
  *            this driver is allowed to allocate. Set this to -1 to allow an
  *            unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread
  *            if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if
  *            ASYN_CANBLOCK is set in asynFlags.
  * \param[in] maxFrames The number of frame buffers to use in driver.
  */
andor3::andor3(const char *portName, int cameraId, int maxBuffers,
               size_t maxMemory, int priority, int stackSize, int maxFrames)
    : ADDriver(portName, 1, NUM_ANDOR3_PARAMS, maxBuffers, maxMemory,
               asynEnumMask, asynEnumMask, 
               ASYN_CANBLOCK,  /* ASYN_CANBLOCK=1 ASYN_MULTIDEVICE=0 */
               1,              /* autoConnect=1 */
               priority, stackSize),
      handle_(0), id_(cameraId), drvBuffers_(NULL), exiting_(0)
{
    static const char *functionName = "andor3";
    int status;

    /* set max frames */
    if(maxFrames == 0) {
        maxFrames_ = 2;
    } else {
        maxFrames_ = maxFrames;
    }
    
    /* create andor specific parameters */
    createParam(Andor3FrameRateString,        asynParamFloat64, &Andor3FrameRate);
    createParam(Andor3PixelEncodingString,    asynParamInt32,   &Andor3PixelEncoding);
    createParam(Andor3FullAOIControlString,   asynParamInt32,   &Andor3FullAOIControl);
    createParam(Andor3BinningString,          asynParamInt32,   &Andor3Binning);
    createParam(Andor3ShutterModeString,      asynParamInt32,   &Andor3ShutterMode);
    createParam(Andor3SoftwareTriggerString,  asynParamInt32,   &Andor3SoftwareTrigger);
    createParam(Andor3SensorCoolingString,    asynParamInt32,   &Andor3SensorCooling);
    createParam(Andor3TempControlString,      asynParamInt32,   &Andor3TempControl);
    createParam(Andor3TempStatusString,       asynParamInt32,   &Andor3TempStatus);
    createParam(Andor3SerialNumberString,     asynParamOctet,   &Andor3SerialNumber);
    createParam(Andor3FirmwareVersionString,  asynParamOctet,   &Andor3FirmwareVersion);
    createParam(Andor3SoftwareVersionString,  asynParamOctet,   &Andor3SoftwareVersion);
    createParam(Andor3ControllerIDString,     asynParamOctet,   &Andor3ControllerID);
    createParam(Andor3OverlapString,          asynParamInt32,   &Andor3Overlap);
    createParam(Andor3ReadoutRateString,      asynParamInt32,   &Andor3ReadoutRate);
    createParam(Andor3ReadoutTimeString,      asynParamFloat64, &Andor3ReadoutTime);
    createParam(Andor3PreAmpGainString,       asynParamInt32,   &Andor3PreAmpGain);
    createParam(Andor3NoiseFilterString,      asynParamInt32,   &Andor3NoiseFilter);
    createParam(Andor3FanSpeedString,         asynParamInt32,   &Andor3FanSpeed);

    featureInfo_ = (featureInfo *)calloc(LAST_ANDOR3_PARAM+1, sizeof(featureInfo));
        
    /* set read-only parameters */
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");

    /* open camera (also allocates frames) */
    status = AT_InitialiseLibrary();
    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: Andor Library initialization failed (%d)\n",
            driverName, functionName, status);
        return;
    }
    AtInitialized++;

    status = connectCamera();
    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s:  camera connection failed (%d)\n",
            driverName, functionName, status);
        return;
    }


    status  = setStringParam(ADManufacturer, "Andor");

    /* register features for change callback (invokes callback to set value)*/

    status |= registerFeature(L"CameraModel",              ATstring, ADModel);
    status |= registerFeature(L"SensorWidth",              ATint,    ADMaxSizeX);
    status |= registerFeature(L"SensorHeight",             ATint,    ADMaxSizeY);
    status |= registerFeature(L"SerialNumber",             ATstring, Andor3SerialNumber);
    status |= registerFeature(L"FirmwareVersion",          ATstring, Andor3FirmwareVersion);
    status |= getFeature(     L"SoftwareVersion",          ATstring, Andor3SoftwareVersion, AT_HANDLE_SYSTEM);
    status |= registerFeature(L"ControllerID",             ATstring, Andor3ControllerID);
    status |= registerFeature(L"FullAOIControl",           ATbool,   Andor3FullAOIControl);

    status  = registerFeature(L"AOIWidth",                 ATint,    ADSizeX);
    status |= registerFeature(L"AOIHeight",                ATint,    ADSizeY);
    status |= registerFeature(L"AOILeft",                  ATint,    ADMinX);
    status |= registerFeature(L"AOITop",                   ATint,    ADMinY);
    status |= registerFeature(L"AOIBinning",               ATenum,   Andor3Binning);

    status |= registerFeature(L"SensorCooling",            ATbool,   Andor3SensorCooling);
    status |= registerFeature(L"TargetSensorTemperature",  ATfloat,  ADTemperature);
    status |= registerFeature(L"SensorTemperature",        ATfloat,  ADTemperatureActual);
    status |= registerFeature(L"TemperatureControl",       ATenum,   Andor3TempControl);
    status |= registerFeature(L"TemperatureStatus",        ATenum,   Andor3TempStatus);
    status |= registerFeature(L"FanSpeed",                 ATenum,   Andor3FanSpeed);
    
    status |= registerFeature(L"CycleMode",                ATenum,   ADImageMode);
    status |= registerFeature(L"ExposureTime",             ATfloat,  ADAcquireTime);
    status |= registerFeature(L"AccumulateCount",          ATint,    ADNumExposures);
    status |= registerFeature(L"FrameCount",               ATint,    ADNumImages);
    status |= registerFeature(L"CameraAcquiring",          ATbool,   ADStatus);

    status |= registerFeature(L"FrameRate",                ATfloat,  Andor3FrameRate);
    status |= registerFeature(L"PixelEncoding",            ATenum,   Andor3PixelEncoding);
    status |= registerFeature(L"ElectronicShutteringMode", ATenum,   Andor3ShutterMode);
    status |= registerFeature(L"Overlap",                  ATbool,   Andor3Overlap);
    status |= registerFeature(L"PixelReadoutRate",         ATenum,   Andor3ReadoutRate);
    status |= registerFeature(L"ReadoutTime",              ATfloat,  Andor3ReadoutTime);
    status |= registerFeature(L"SimplePreAmpGainControl",  ATenum,   Andor3PreAmpGain);
    status |= registerFeature(L"SpuriousNoiseFilter",      ATbool,   Andor3NoiseFilter);

    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: failed to register all features for camera %d\n",
            driverName, functionName, id_);
    }

    startEvent_ = epicsEventCreate(epicsEventEmpty);

    /* launch image read task */
    epicsThreadCreate("Andor3ImageTask", 
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      c_imagetask, this);

    /* launch temp read task */
    epicsThreadCreate("Andor3TempTask", 
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      c_temptask, this);

    /* shutdown on exit */
    epicsAtExit(c_shutdown, this);

    /* Hard coded values for test weekend */
    status  = AT_SetBool(handle_, L"Overlap", AT_TRUE);
    status |= AT_SetBool(handle_, L"SpuriousNoiseFilter", AT_TRUE);

    status |= AT_SetEnumIndex(handle_, L"PixelEncoding", 2);
    status |= AT_SetEnumIndex(handle_, L"SimplePreAmpGainControl", 2);
 
    status |= AT_SetEnumIndex(handle_, L"PixelReadoutRate", 3);
    
    if(status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: failed to set hard coded values: %d\n", 
            driverName, functionName, status);
    }
}



/* Code for iocsh registration */
static const iocshArg andor3ConfigArg0 = {"Port name", iocshArgString};
static const iocshArg andor3ConfigArg1 = {"CameraId", iocshArgInt};
static const iocshArg andor3ConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg andor3ConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg andor3ConfigArg4 = {"priority", iocshArgInt};
static const iocshArg andor3ConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg andor3ConfigArg6 = {"maxFrames", iocshArgInt};
static const iocshArg * const andor3ConfigArgs[] =  {&andor3ConfigArg0,
                                                     &andor3ConfigArg1,
                                                     &andor3ConfigArg2,
                                                     &andor3ConfigArg3,
                                                     &andor3ConfigArg4,
                                                     &andor3ConfigArg5,
                                                     &andor3ConfigArg6};
static const iocshFuncDef configAndor3 = {"andor3Config", 7, andor3ConfigArgs};
static void configAndor3CallFunc(const iocshArgBuf *args)
{
    andor3Config(args[0].sval, args[1].ival, args[2].ival,  args[3].ival, 
                 args[4].ival, args[5].ival, args[6].ival);
}

static void andor3Register(void)
{

    iocshRegister(&configAndor3, configAndor3CallFunc);
}

extern "C" {
epicsExportRegistrar(andor3Register);
}
