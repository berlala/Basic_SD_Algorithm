@echo off

set "VSCMD_START_DIR=%CD%"

call "setup_msvc150.bat"

call "%VS150COMNTOOLS%..\..\VC\Auxiliary\Build\VCVARSALL.BAT" AMD64


call  "\\CND8302MKS\D$\Program Files\MATLAB\R2018b\bin\win64\checkMATLABRootForDriveMap.exe" "\\CND8302MKS\D$\Program Files\MATLAB\R2018b"  > mlEnv.txt
for /f %%a in (mlEnv.txt) do set "%%a"\n
cd .

if "%1"=="" (nmake MATLAB_ROOT=%MATLAB_ROOT% ALT_MATLAB_ROOT=%ALT_MATLAB_ROOT% MATLAB_BIN=%MATLAB_BIN% ALT_MATLAB_BIN=%ALT_MATLAB_BIN%  -f find_P_rtw.mk all) else (nmake MATLAB_ROOT=%MATLAB_ROOT% ALT_MATLAB_ROOT=%ALT_MATLAB_ROOT% MATLAB_BIN=%MATLAB_BIN% ALT_MATLAB_BIN=%ALT_MATLAB_BIN%  -f find_P_rtw.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
An_error_occurred_during_the_call_to_make
