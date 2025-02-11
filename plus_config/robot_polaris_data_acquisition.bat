@echo off

title Robot-Polaris data acquisition

echo This batch file aims to collect transformation data from the robot and from polaris to calculate the spatial calibration between them

echo.

pause

IF EXIST data_acquisition*.mha DEL /F data_acquisition*.mha
IF EXIST data_acquisition*.xml DEL /F data_acquisition*.xml

echo.

echo going to D:\Programas\PlusApp\bin

D:

cd D:\Programas\PlusApp\bin

echo.

echo You have 10 seconds before PlusServer starts collecting the data
echo Press any key to start the counting...
pause >nul

echo.

timeout /t 10

echo.

PlusServer --config-file=C:\Dev\RobotPolarisCalibration\plus_config\robot_polaris_data_acquisition.xml

C:
cd C:\Dev\RobotPolarisCalibration\plus_config

rename "data_acquisition*.mha" "data_acquisition.mha"
del "data_acquisition*.xml"

echo Starting Temporal Calibration:

D:

cd D:\Programas\PlusApp\bin

echo.





echo.
echo ########################################################
echo Data acquisition completed!!!
echo.
set /p Input=Write the matlab data file for the recorded data: 
echo.

pause

echo The file data is: "%Input%"


echo Press any key to exit
pause >nul