#include <stdio.h>
#include <stdlib.h>

#include <atcore.h>

#define MAX_STRING_LEN 64

int main(int argc, char *argv[])
{
     int   device;
     AT_WC feature[MAX_STRING_LEN];
     AT_WC indexNameW[MAX_STRING_LEN];
     char  indexName[MAX_STRING_LEN];
     AT_H  handle;
     int   status;
     int   count;
     int   x;


     device = atoi(argv[1]);
     mbstowcs(feature, argv[2], MAX_STRING_LEN);

     AT_InitialiseLibrary();
     AT_Open(device, &handle);

     if(status = AT_GetEnumCount(handle, feature, &count)) {
       printf("Error for %s (%d)\n", argv[2], status);
       return 1;
     }
     for(x = 0; x < count; x++) {
          AT_GetEnumStringByIndex(handle, feature, x, indexNameW, MAX_STRING_LEN);
          wcstombs(indexName, indexNameW, MAX_STRING_LEN);
          printf("%d\t%s\n", x, indexName);
     }

     AT_Close(handle);
     AT_FinaliseLibrary();
}
