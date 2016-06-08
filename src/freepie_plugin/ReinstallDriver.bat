@echo off
call SetDriverVars.bat

echo "(Re)Installing PSMoveService OpenVR driver..."
IF NOT EXIST "%INSTALL_DIR%\bin\%PLATFORM%" mkdir "%INSTALL_DIR%\bin\%PLATFORM%"
copy %BUILD_DIR%\openvr_plugin\Release\driver_psmoveservice.dll "%INSTALL_DIR%\bin\%PLATFORM%\driver_psmove.dll"
copy %BUILD_DIR%\psmoveclient\Release\PSMoveClient.dll "%INSTALL_DIR%\bin\%PLATFORM%"
"%STEAMVR_RUNTIME_DIR%\bin\win32\vrpathreg" adddriver "%INSTALL_DIR%"
xcopy /s /i /y "resources" "%STEAMVR_RUNTIME_DIR%\drivers\psmove\resources"

echo "Done"
pause
