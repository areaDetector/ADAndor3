rem start medm -x -macro "P=13ANDOR3:, R=cam1:" andor3.adl &
rem Must put the path to the Andor SDK3 in the PATH so the application can find needed DLLs
set PATH=C:\Program Files\Andor SDK3\win32;%PATH%
..\..\bin\win32-x86\andor3App st.cmd
pause

