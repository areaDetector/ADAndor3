#include "atcore.h"
#include <iostream> 
using namespace std;

#define NEO_WIDTH 2560
#define NEO_HEIGHT 2160

static int AT_EXP_CONV AOICallback(AT_H handle, const AT_WC *feature, void *context)
{
  int status=0;
  AT_64 width, left, height, top;
  int binning;
  wcout << "AOICallback, entry feature = " << feature << endl;

  cout << "  calling AT_GetInt for AOIWidth" << endl;
  status |= AT_GetInt(handle, L"AOIWidth",  &width);
  cout << "  calling AT_GetInt for AOILeft" << endl;
  status |= AT_GetInt(handle, L"AOILeft",   &left);
  cout << "  calling AT_GetInt for AOIHeight" << endl;
  status |= AT_GetInt(handle, L"AOIHeight", &height);
  cout << "  calling AT_GetInt for AOITop" << endl;
  status |= AT_GetInt(handle, L"AOITop",    &top);
  cout << "  calling AT_GetEnumIndex for AOIBinning" << endl;
  status |= AT_GetEnumIndex(handle, L"AOIBinning", &binning);
  if (status) {
    cout << "  error getting values, error= " << status << endl;
    return status;
  }
  cout << "width=" << width << " left=" << left 
       << " height=" << height << " top=" << top << endl;
  return 0;
}

int main(int argc, char* argv[])
{
  int i_retCode;
  int setWidthFirst=0;
  
  if (argc > 1) setWidthFirst=1;

  cout << "Initialising ..." << endl << endl;
  i_retCode = AT_InitialiseLibrary();
  if (i_retCode != AT_SUCCESS) {
    cout << "Error initialising library" << endl << endl;
    return -1;
  }
  AT_64 iNumberDevices = 0;
  AT_GetInt(AT_HANDLE_SYSTEM, L"Device Count", &iNumberDevices);
  if (iNumberDevices <= 0) {
    cout << "No cameras detected"<<endl;
    return -1;
  }
  AT_H Hndl;
  i_retCode = AT_Open(0, &Hndl);
  if (i_retCode != AT_SUCCESS) {
    cout << "Error condition, could not initialise camera" << endl << endl;
  }
  else {
    cout << "Successfully initialised camera" << endl << endl;
  }
  i_retCode = AT_RegisterFeatureCallback(Hndl, L"AOIBinning", AOICallback, 0);
  i_retCode = AT_RegisterFeatureCallback(Hndl, L"AOIWidth", AOICallback, 0);
  i_retCode = AT_RegisterFeatureCallback(Hndl, L"AOILeft", AOICallback, 0);
  i_retCode = AT_RegisterFeatureCallback(Hndl, L"AOIHeight", AOICallback, 0);
  i_retCode = AT_RegisterFeatureCallback(Hndl, L"AOITop", AOICallback, 0);
  int numBin = 5;
  int binValues[] = {1,2,3,4,8};
  AT_64 width, height;
  for (int i=0; i<numBin; i++) {
    if (setWidthFirst) {
      cout << "Setting binning to 1" << endl;
      i_retCode = AT_SetEnumIndex(Hndl, L"AOIBinning", 0);
      cout << "Setting binning 1 returned " << i_retCode << endl;
      width = (NEO_WIDTH / binValues[i]) * binValues[i];
      height = (NEO_HEIGHT / binValues[i]) * binValues[i];
      i_retCode= AT_SetInt(Hndl, L"AOIWidth", width); ;
      cout << "Setting width to " << width << " returned " << i_retCode << endl;
      i_retCode= AT_SetInt(Hndl, L"AOIHeight", height); ;
      cout << "Setting height to " << height << " returned " << i_retCode << endl;
    }
    cout << "Setting binning to " << binValues[i] << endl;
    i_retCode= AT_SetEnumIndex(Hndl, L"AOIBinning", i);
    cout << "Setting binning to " << binValues[i] << " returned " << i_retCode << endl << endl;
  }
  AT_Close(Hndl);
  AT_FinaliseLibrary();
  return 0;
}


