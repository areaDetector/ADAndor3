#include "atcore.h"
#include <iostream> 
#include <stdlib.h>
#include <time.h>

using namespace std;

#define EXTRACTLOWPACKED(SourcePtr) ( (SourcePtr[0] << 4) + (SourcePtr[1] & 0xF) )
#define EXTRACTHIGHPACKED(SourcePtr) ( (SourcePtr[2] << 4) + (SourcePtr[1] >> 4) )

#define NUM_BUFFERS 10

int main(int argc, char* argv[])
{
  int i_retCode;
  AT_64 iNumberDevices = 0;
  AT_H Hndl;
  double frameRate=48.0;
  double maxFrameRate;
  double transferRate;
  int numFrames=320;
  double exposureTime = 0.001;
  AT_64 imageSizeBytes;
  int bufferSize;
  AT_U8 **buffers;
  AT_U8 *image;
  time_t tStart, tEnd;
  int i;
  
  if (argc > 3) exposureTime = atof(argv[3]);
  if (argc > 2) frameRate    = atof(argv[2]);
  if (argc > 1) numFrames    = atoi(argv[1]);

  cout << "frameRate=" << frameRate << " numFrames=" << numFrames << endl;
  
  cout << "Initialising ..." << endl;
  i_retCode = AT_InitialiseLibrary();
  if (i_retCode != AT_SUCCESS) {
    cout << "Error initialising library, error=" << i_retCode << endl;
    return -1;
  }
  AT_GetInt(AT_HANDLE_SYSTEM, L"Device Count", &iNumberDevices);
  if (iNumberDevices <= 0) {
    cout << "No cameras detected, error=" << i_retCode <<endl;
    return -1;
  }
  i_retCode = AT_Open(0, &Hndl);
  if (i_retCode != AT_SUCCESS) {
    cout << "Error condition, could not initialise camera, error=" << i_retCode << endl;
    return -1;
  }

  //Set the pixel Encoding to the desired settings Mono16 Data
  i_retCode = AT_SetEnumeratedString(Hndl, L"PixelEncoding", L"Mono16");
  if (i_retCode != AT_SUCCESS) {
    cout << "Error condition, could not set pixel encoding to Mono16" << endl;
  }
  
  //Set global shutter
  i_retCode = AT_SetEnumeratedString(Hndl, L"ElectronicShutteringMode", L"Global");
  if (i_retCode != AT_SUCCESS) {
    cout << "Error condition, could not set shutter mode to Global" << endl;
  }
  
  //Set the pixel Readout Rate to fastest
  i_retCode = AT_SetEnumeratedString(Hndl, L"PixelReadoutRate", L"280 MHz");
  if (i_retCode != AT_SUCCESS) {
    cout << "Error condition, could not set readout rate to 280 MHz" << endl;
  }

  //Set the pre-amp gain control
  i_retCode = AT_SetEnumeratedString(Hndl, L"SimplePreAmpGainControl", L"16-bit (low noise & high well capacity)");
  if (i_retCode != AT_SUCCESS) {
    cout << "Error condition, could not set pre-amp gain to 16-bit" << endl;
  }

  //Set the Cycle mode to fixed
  i_retCode = AT_SetEnumeratedString(Hndl, L"CycleMode", L"Fixed");
  if (i_retCode != AT_SUCCESS) {
    cout << "Error condition, could not set cycle mode to Fixed" << endl;
  }

  //Set the frame count to numFrames
  i_retCode = AT_SetInt(Hndl, L"FrameCount", numFrames);
  if (i_retCode != AT_SUCCESS) {
    cout << "Error condition, could not set frame count=" << numFrames << endl;
  }

  //Set the exposure time
  i_retCode = AT_SetFloat(Hndl, L"Exposure Time", exposureTime);
  if (i_retCode != AT_SUCCESS) {
    cout << "Error condition, could not set exposure time to " << exposureTime << endl;
  }

  //Set the frame rate to frameRate
  i_retCode = AT_GetFloatMax(Hndl, L"FrameRate", &maxFrameRate);
  cout << "Exposure time=" << exposureTime << " maximum frame rate=" << maxFrameRate << endl;
  i_retCode = AT_SetFloat(Hndl, L"FrameRate", frameRate);
  if (i_retCode != AT_SUCCESS) {
    cout << "Error condition, could not set frame rate=" << frameRate << endl;
    i_retCode = AT_GetFloat(Hndl, L"FrameRate", &frameRate);
    cout << "  actual frame rate=" << frameRate << endl;
  }

  //Get the number of bytes required to store one frame
  AT_GetInt(Hndl, L"Image Size Bytes", &imageSizeBytes);
  bufferSize = (int)(imageSizeBytes);

  // Allocate and queue buffers
  buffers = (AT_U8 **)calloc(NUM_BUFFERS, sizeof(AT_U8 *));
  for (i=0; i<NUM_BUFFERS; i++) {
    #ifdef _WIN32
       buffers[i] = (AT_U8*)_aligned_malloc(bufferSize, 8);
    #else
      /* allocate 8 byte aligned buffer */
      posix_memalign((void **)&buffers[i], 8, bufferSize);
    #endif
    i_retCode = AT_QueueBuffer(Hndl, buffers[i], bufferSize);
  }

  i_retCode = AT_GetFloat(Hndl, L"MaxInterfaceTransferRate", &transferRate);
  cout << "Max. interface transfer rate=" << transferRate << endl;

  cout << "Starting acquisition ..." << endl;
  tStart = time(NULL);

  //Start the Acquisition running
  AT_Command(Hndl, L"Acquisition Start");

  for (i=0; i<numFrames; i++) {
    i_retCode = AT_WaitBuffer(Hndl, &image,  &bufferSize, 2000);
    tEnd = time(NULL);
    
    if (i_retCode != AT_SUCCESS) {
      cout << "Error condition, failure return from AT_WaitBuffer=" << i_retCode << endl;
      break;
    }
    cout << "Got frame " << i << " time=" << tEnd-tStart << endl;
    i_retCode = AT_QueueBuffer(Hndl, image, bufferSize);
    if (i_retCode != AT_SUCCESS) {
      cout << "Error condition, failure return from AT_QueueBuffer=" << i_retCode << endl;
    }
  }
  AT_Close(Hndl);
  AT_FinaliseLibrary();
  return 0;
}


