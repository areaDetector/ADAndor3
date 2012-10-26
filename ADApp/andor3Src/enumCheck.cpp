#include <stdio.h>
#include <stdlib.h>

#include <atcore.h>


int main(int argc, char *argv[])
{
     int   device;
     AT_WC feature[32];
     AT_WC indexNameW[32];
     char  indexName[32];
     AT_H  handle;
     int   status;
     int   count;
     int   x;


     device = atoi(argv[1]);
     mbstowcs(feature, argv[2], 32);

     AT_InitialiseLibrary();
     AT_Open(device, &handle);

     if(status = AT_GetEnumCount(handle, feature, &count)) {
       printf("Error for %s (%d)\n", argv[2], status);
       return 1;
     }
     for(x = 0; x < count; x++) {
          AT_GetEnumStringByIndex(handle, feature, x, indexNameW, 32);
          wcstombs(indexName, indexNameW, 32);
          printf("%d\t%s\n", x, indexName);
     }

     AT_Close(handle);
     AT_FinaliseLibrary();
}
