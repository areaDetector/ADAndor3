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

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsExit.h>

#include <atcore.h>

#include "ADDriver.h"

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


class andor3 : public ADDriver {
public:
    andor3(const char *portName, int cameraId, int maxBuffers, 
           size_t maxMemory, int priority, int stackSize, int maxFrames);

    /* override bADDriver methods */ 
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

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
    int Andor3SensorCooling;
    int Andor3ShutterMode;
    int Andor3SoftwareTrigger;
    int Andor3TempControl;
    int Andor3TempStatus;
    int Andor3Last;
    #define LAST_ANDOR3_PARAM Andor3Last
private:
    int setFeature(const AT_WC *feature, Andor3FeatureType type,
                   int paramIndex);
    int registerFeature(const AT_WC *feature, Andor3FeatureType type,
                        int paramIndex);

    int updateAOI(int set);
    
    int allocateBuffers();
    int freeBuffers();

    int connectCamera();
    int disconnectCamera();

    AT_H    handle;
    int     id;
    int     maxFrames;
    AT_U8 **drvBuffers;    
    AT_64   imageSize;
  epicsEventId start;
};
#define NUM_ANDOR3_PARAMS (&LAST_ANDOR3_PARAM - &FIRST_ANDOR3_PARAM + 1)

/* Andor3 driver specific parameters */
#define Andor3FrameRateString        "FRAME_RATE"        /* asynFloat64  rw */
#define Andor3PixelEncodingString    "PIXEL_ENCODING"    /* asynInt32    rw */
#define Andor3SensorCoolingString    "SENSOR_COOLING"    /* asynInt32    rw */
#define Andor3ShutterModeString      "A3_SHUTTER_MODE"   /* asynInt32    rw */
#define Andor3SoftwareTriggerString  "SOFTWARE_TRIGGER"  /* asynInt32    wo */
#define Andor3TempControlString      "TEMP_CONTROL"      /* asynInt32    rw */
#define Andor3TempStatusString       "TEMP_STATUS"       /* asynInt32    ro */

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

typedef struct {
  andor3*           camera;
  Andor3FeatureType type;
  int               paramIndex;
} featureInfo;


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
    if(this->handle) {
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
    int            status;
    AT_U8         *image;
    int            size;
    int            acquire;

    this->lock();

    while(1) {
        this->unlock();

	getIntegerParam(ADAcquire, &acquire);
	if(!acquire) {
	    AT_Flush(this->handle);

	    epicsEventWait(this->start);
	    
	    AT_Flush(this->handle);
	    for(int x=0; x<maxFrames; x++) {
		if(drvBuffers[x]) {
		    status = AT_QueueBuffer(this->handle, drvBuffers[x],
					    imageSize);
		    if(status) {
			printf("Queue error: %d\n", status);
		    }
		}
	    }
	    AT_Command(this->handle, L"AcquisitionStart");
	}
        status = AT_WaitBuffer(this->handle, &image, &size, AT_INFINITE);
        epicsTimeGetCurrent(&imageStamp);

        this->lock();

        if(status != AT_SUCCESS) {
            printf("error %d\n", status);
	    continue;
        } else {
            int mode;
            int total;
	    int number;
            int count;
            int callback;

            getIntegerParam(ADNumImagesCounter, &number);
            number++;
            setIntegerParam(ADNumImagesCounter, number);
            callParamCallbacks();

            getIntegerParam(NDArrayCallbacks, &callback);
            if(callback) {
                NDArray *pImage;
                int      dims[2];

                getIntegerParam(NDArraySizeX, &dims[0]);
                getIntegerParam(NDArraySizeY, &dims[1]);

                pImage = this->pNDArrayPool->alloc(2, dims, NDUInt16, 0, NULL);
                if(pImage) {
                    int encoding;

                    getIntegerParam(NDArrayCounter, &count);
                    count++;
                    setIntegerParam(NDArrayCounter, count);
                    callParamCallbacks();

                    pImage->uniqueId = count;
                    pImage->timeStamp = 631152000 + imageStamp.secPastEpoch +
                        (imageStamp.nsec / 1.0e9);

                    getIntegerParam(Andor3PixelEncoding, &encoding);
                    if(encoding == ATMono12 || encoding == ATMono16) {
			AT_64 stride;
			int   x_len;
			AT_U8 *p;

			AT_GetInt(this->handle, L"AOIStride", &stride);
			x_len = dims[0] * 2;
			p = (AT_U8 *)pImage->pData;

			for(int x = 0; x < size; x += stride) {
			    memcpy(p, image+x, x_len);
			    p += x_len;
			}
                    } else if(encoding == ATMono12Packed) {
                        // Probably broken -- works with Sim cam.
                        // not tested on real camera -- needs line by line like
                        // above
                        AT_U8 *enc = image;
                        unsigned short *dec = (unsigned short*)pImage->pData;

                        for(int x = 0; x < size; x+=3) {
                            *dec     = (*enc << 4) + (*(enc+1) & 0xf);
                            *(dec+1) = (*(enc+2)<<4) + ((*(enc+1) >> 4) & 0xf);

                            enc += 3;
                            dec += 2;
                        }
                    }

                    this->getAttributes(pImage->pAttributeList);

                    this->unlock();
                    doCallbacksGenericPointer(pImage, NDArrayData, 0);
                    this->lock();

                    pImage->release();
                }
            }
	    getIntegerParam(ADImageMode, &mode);
	    getIntegerParam(ADNumImages, &total);
	    if(mode == ATFixed && number == total) {
		setShutter(0);
		AT_Command(this->handle, L"AcquisitionStop");
		setIntegerParam(ADAcquire, 0);
	    }
	    callParamCallbacks();
	}

        status = AT_QueueBuffer(this->handle, image, size);
	if(status) {
	    printf("Queue 2: %d\n", status);
	}
    }
}

void andor3::tempTask(void)
{
    int status;

    while(1) {
	status = getFeature(L"SensorTemperature", ATfloat, ADTemperatureActual);
	status |= getFeature(L"SensorCooling", ATbool, Andor3SensorCooling);
	status |= getFeature(L"TemperatureStatus", ATenum, Andor3TempStatus);

	if(status) {
	    printf("Temperature read error\n");
	}
	epicsThreadSleep(1.0);
    }
}

int andor3::getFeature(const AT_WC *feature, Andor3FeatureType type,
                       int paramIndex)
{
    static const char* functionName = "getFeature";

    int     status = 0;
    char    featureName[64];
    AT_64   i_value;
    double  d_value;
    AT_BOOL b_value;
    int     index;
    int     length;
    AT_WC  *wide;
    char   *str;

    wcstombs(featureName, feature, 64);

    /* get feature value to paramIndex */
    switch(type) {
    case ATint:
        status = AT_GetInt(this->handle, feature, &i_value);
        if(status == AT_SUCCESS) {
            status = setIntegerParam(paramIndex, (int)i_value);
            if(status) {
                status = -1;
            }
        }
        break;
    case ATfloat:
        status = AT_GetFloat(this->handle, feature, &d_value);
        if(status == AT_SUCCESS) {
            status = setDoubleParam(paramIndex, d_value);
            if(status) {
                status = -1;
            }
        }
        break;
    case ATbool:
        status = AT_GetBool(this->handle, feature, &b_value);
        if(status == AT_SUCCESS) {
            status = setIntegerParam(paramIndex, (int)b_value);
            if(status) {
                status = -1;
            }
        }
        break;
    case ATenum:
        status = AT_GetEnumIndex(this->handle, feature, &index);
        if(status == AT_SUCCESS) {
            status = setIntegerParam(paramIndex, index);
            if(status) {
                status = -1;
            }
        }
        break;
    case ATstring:
        status = AT_GetStringMaxLength(this->handle, feature, &length);
        if(status == AT_SUCCESS) {
            length++;

            wide = new AT_WC[length];
            str  = new char[length];

            status = AT_GetString(this->handle, feature, wide, length);
            if(status == AT_SUCCESS) {
                wcstombs(str, wide, length);
                status = setStringParam(paramIndex, str);
                if(status) {
                    status = -1;
                }
            }
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
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: unable to set parameter index %d\n",
                  driverName, functionName, paramIndex);
        status = AT_ERR_NOTWRITABLE;
    } else if(status != AT_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
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

    char   featureName[64];
    int    status=0;
    int    i_value;
    AT_64  i_min;
    AT_64  i_max;
    double d_value;
    double d_min;
    double d_max;


    wcstombs(featureName, feature, 64);

    /* get feature value to paramIndex */
    switch(type) {
    case ATint:
        status = getIntegerParam(paramIndex, &i_value);
        if(status) {
            status = -1;
        } else {
            status  = AT_GetIntMax(this->handle, feature, &i_max);
            status |= AT_GetIntMin(this->handle, feature, &i_min);
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
                status = AT_SetInt(this->handle, feature, (AT_64)i_value);
            }
        }
        break;
    case ATfloat:
        status = getDoubleParam(paramIndex, &d_value);
        if(status) {
            status = -1;
        } else {
            status  = AT_GetFloatMax(this->handle, feature, &d_max);
            status |= AT_GetFloatMin(this->handle, feature, &d_min);
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
                status = AT_SetFloat(this->handle, feature, d_value);
            }
        }
        break;
    case ATbool:
        status = getIntegerParam(paramIndex, &i_value);
        if(status) {
            status = -1;
        } else {
            status = AT_SetBool(this->handle, feature, 
				i_value ? AT_TRUE : AT_FALSE);
        }
        break;
    case ATenum:
        status = getIntegerParam(paramIndex, &i_value);
        if(status) {
            status = -1;
        } else {
            status = AT_SetEnumIndex(this->handle, feature, i_value);
        }
        break;
    case ATstring:
        status = 2;
        break;
    }

    /* error messages */
    if(status == -1) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: unable to get parameter index %d\n",
                  driverName, functionName, paramIndex);
        status = AT_ERR_NOTWRITABLE;
    } else if(status != AT_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
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
    char featureName[64];

    wcstombs(featureName, feature, 64);

    featureInfo *info = (featureInfo *)malloc(sizeof(featureInfo));
    if(info) {
        info->camera = this;
        info->type = type;
        info->paramIndex = paramIndex;

        status = AT_RegisterFeatureCallback(this->handle, feature, 
                                            c_getfeature, info);
    }

    if(status == -1) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: unable to allocate memory for featureInfo %s\n",
                  driverName, functionName, featureName);
        status = AT_ERR_NOTWRITABLE;
    } else if(status != AT_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: unable to register feature %s (%d)\n",
                  driverName, functionName, featureName, status);   
    }
    return status;
}

/* if set  true  -> update camera from parameters 
           false -> update parameters from camera */
int andor3::updateAOI(int set)
{
    int   status;
    AT_64 sizeX;
    AT_64 sizeY;
    AT_64 sizeI;


    if(set) {
        status  = getIntegerParam(ADSizeX, (int *)&sizeX);
        status |= getIntegerParam(ADSizeY, (int *)&sizeY);
    } else {
        status  = AT_GetInt(this->handle, L"AOIWidth", &sizeX);
        status |= AT_GetInt(this->handle, L"AOIHeight", &sizeY);
    }
    if(!status) {
        if(set) {
            status |= setFeature(L"AOIWidth", ATint, ADSizeX);
            status |= setFeature(L"AOIHeight", ATint, ADSizeY);
        } else {
            status |= setIntegerParam(ADSizeX, sizeX);
            status |= setIntegerParam(ADSizeY, sizeY);
        }   

        /* set NDArray parameters */
        status  = setIntegerParam(NDArraySizeX, sizeX);
        status |= setIntegerParam(NDArraySizeY, sizeY);
        status |= setIntegerParam(NDArraySize, (sizeX * sizeY * 2));
    } else {
        /* print error message */
        return status;
    }

    /* reallocate image buffers if size changed */
    status = AT_GetInt(this->handle, L"ImageSizeBytes", &sizeI);
    if(status == AT_SUCCESS && sizeI != imageSize) {
        allocateBuffers();
    }
    return status;
}

int andor3::allocateBuffers(void)
{
    int   status;
    AT_64 size;

    freeBuffers();

    status = AT_GetInt(this->handle, L"ImageSizeBytes", &size);
    if(status != AT_SUCCESS) {
        size = 11059200; /* 16 bit full size */
        status = AT_SUCCESS;
    }
    this->imageSize = size;

    this->drvBuffers = (AT_U8 **)calloc(this->maxFrames, sizeof(AT_U8 *));
    if(this->drvBuffers) {
        for(int x = 0; x < this->maxFrames; x++) {
            #ifdef _WIN32
               drvBuffers[x] = (AT_U8*)_aligned_malloc(size, 8);
            #else
                /* allocate 8 byte aligned buffer */
                if(!posix_memalign((void **)&drvBuffers[x], 8, size)) {
                    //status |= AT_QueueBuffer(this->handle,
                    //                         drvBuffers[x], (int)size);
                } else {   
                    drvBuffers[x] = NULL;
                }
            #endif
        }
    }

    if(status != AT_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:allocateBuffers: Failed to allocate and queue buffers\n",
                  driverName);
    }   
    return status;
}

int andor3::freeBuffers()
{
    int status;

    status = AT_Flush(this->handle);
    if(this->drvBuffers) {
        for(int x = 0; x < this->maxFrames; x++) {
            if(drvBuffers[x]) {
                #ifdef _WIN32
                    _aligned_free(drvBuffers[x]);
                #else
                    free(drvBuffers[x]);
                #endif
                drvBuffers[x] = NULL;
            }
        }
        free(drvBuffers);
        drvBuffers = NULL;
    }
    return status;
}

    
int andor3::connectCamera(void)
{
    static const char *functionName = "connectCamera";

    int status = AT_SUCCESS;


    /* disconnect any connected camera first */
    disconnectCamera();

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: connecting camera %d\n",
              driverName, functionName, this->id);

    /* open handle to camera */
    status = AT_Open(this->id, &this->handle);
    if(status != AT_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: unable to open camera %d\n",
                  driverName, functionName, this->id);
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


    if(!this->handle) {
        return AT_SUCCESS;
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: disconnecting camera %d\n",
              driverName, functionName, this->id);

    status = AT_GetBool(this->handle, L"CameraAcquiring", &acquiring);
    if(status == AT_SUCCESS && acquiring) {
        status |= AT_Command(this->handle, L"Acquisition Stop");
    }

    status |= freeBuffers();
    status |= AT_Close(this->handle);

    if(status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: error closing camera %d\n",
                  driverName, functionName, this->id);
    }

    this->handle = 0;
    return status;
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
	    epicsEventSignal(this->start);
        } else {
            setShutter(0);
            status = AT_Command(this->handle, L"AcquisitionStop");
        }
    }
    else if(index == ADImageMode) {
        status = setFeature(L"CycleMode", ATenum, ADImageMode);        
    }
    else if(index == ADTriggerMode) {
        status = setFeature(L"TriggerMode", ATenum, ADTriggerMode);
    }
    /*else if(index == ADNumExposures) {
        status = setFeature(L"AccumulateCount", ATint, ADNumExposures);
    }*/
    else if(index == ADNumImages) {
        status = setFeature(L"FrameCount", ATint, ADNumImages);
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
            status = AT_Command(this->handle, L"SoftwareTrigger");
            setIntegerParam(Andor3SoftwareTrigger, 0);
        }
    }
    else if(index == Andor3TempControl) {
	status = setFeature(L"TemperatureControl", ATenum, Andor3TempControl);
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

/** Constructor for Pilatus driver; most parameters are simply passed to
  * ADDriver::ADDriver.
  *
  * After calling the base class constructor this method creates a thread to
  * collect the images from the detector and sets reasonable default values for
  * the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  *
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] camerId The id number of the Andor camera (see listdevices
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
               0, 0,           /* No interfaces beyond those in ADDriver.cpp */
               ASYN_CANBLOCK,  /* ASYN_CANBLOCK=1 ASYN_MULTIDEVICE=0 */
               1,              /* autoConnect=1 */
               priority, stackSize),
      handle(0), id(cameraId)
{
    static const char *functionName = "andor3";
    int status;

    /* set max frames */
    if(maxFrames == 0) {
        this->maxFrames = 2;
    } else {
        this->maxFrames = maxFrames;
    }
    
    this->drvBuffers = NULL;

    /* set read-only parameters */
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");

    /* create andor specific parameters */
    createParam(Andor3FrameRateString, 
                asynParamFloat64, &Andor3FrameRate);
    createParam(Andor3PixelEncodingString,
                asynParamInt32, &Andor3PixelEncoding);
    createParam(Andor3SensorCoolingString,
		asynParamInt32, &Andor3SensorCooling);
    createParam(Andor3ShutterModeString,
                asynParamInt32, &Andor3ShutterMode);
    createParam(Andor3SoftwareTriggerString,
                asynParamInt32, &Andor3SoftwareTrigger);
    createParam(Andor3TempControlString,
		asynParamInt32, &Andor3TempControl);
    createParam(Andor3TempStatusString,
		asynParamInt32, &Andor3TempStatus);

    /* open camera (also allocates frames) */
    status = AT_InitialiseLibrary();
    if(status != AT_SUCCESS) {
        printf("%s:%s: Andor Library initialization failed (%d)\n",
               driverName, functionName, status);
        return;
    }
    AtInitialized++;

    status = connectCamera();
    if(status != AT_SUCCESS) {
        printf("%s:%s:  camera connection failed (%d)\n",
               driverName, functionName, status);
        return;
    }

    /* set ReadOnce parameters from feature values */
    status  = setStringParam(ADManufacturer, "Andor");
    status |= getFeature(L"CameraModel", ATstring, ADModel);
    status |= getFeature(L"SensorWidth", ATint, ADMaxSizeX);
    status |= getFeature(L"SensorHeight", ATint, ADMaxSizeY);

    if(status != AT_SUCCESS) {
        printf("%s:%s: failed to read parameters from camera %d\n",
               driverName, functionName, this->id);
        return;
    }

    /* register features for change callback (invokes callback to set value)*/
    status  = registerFeature(L"AOIWidth", ATint, ADSizeX);
    status |= registerFeature(L"AOIHeight", ATint, ADSizeY);
    status |= registerFeature(L"AOILeft", ATint, ADMinX);
    status |= registerFeature(L"AOITop", ATint, ADMinY);

    status |= registerFeature(L"TargetSensorTemperature", ATfloat,
                              ADTemperature);
    status |= registerFeature(L"SensorTemperature", ATfloat,
                              ADTemperatureActual);
    
    status |= registerFeature(L"CycleMode", ATenum, ADImageMode);
    status |= registerFeature(L"ExposureTime", ATfloat, ADAcquireTime);
    //status |= registerFeature(L"AccumulateCount", ATint,  ADNumExposures);
    status |= registerFeature(L"FrameCount", ATint, ADNumImages);
    status |= registerFeature(L"CameraAcquiring", ATbool, ADStatus);

    status |= registerFeature(L"FrameRate", ATfloat, Andor3FrameRate);
    status |= registerFeature(L"PixelEncoding", ATenum, Andor3PixelEncoding);
    status |= registerFeature(L"SensorCooling", ATbool, Andor3SensorCooling);
    status |= registerFeature(L"ElectronicShutteringMode", ATenum,
                              Andor3ShutterMode);
    status |= registerFeature(L"TemperatureControl", ATenum,
			      Andor3TempControl);
    status |= registerFeature(L"TemperatureStatus", ATenum,
			      Andor3TempStatus);

    if(status != AT_SUCCESS) {
        printf("%s:%s: failed to register all features for camera %d\n",
               driverName, functionName, this->id);
        //return;
    }

    this->start = epicsEventCreate(epicsEventEmpty);

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
    status  = AT_SetBool(this->handle, L"Overlap", AT_TRUE);
    status |= AT_SetBool(this->handle, L"SpuriousNoiseFilter", AT_TRUE);

    status |= AT_SetEnumIndex(this->handle, L"PixelEncoding", 2);
    status |= AT_SetEnumIndex(this->handle, L"SimplePreAmpGainControl", 2);
 
    status |= AT_SetEnumIndex(this->handle, L"PixelReadoutRate", 3);
    
    if(status) {
	printf("Failed to set hard coded values: %d\n", status);
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
