@echo off
cd /d C:\works\C\ra6m4
set CMAKE=C:\Users\二手精品莆田鞋\.renesas\platform\cmake\3.31.8\cmake-3.31.8-windows-x86_64\bin\cmake.exe
set NINJA=C:\Users\二手精品莆田鞋\.renesas\platform\ninja-build\ninja.exe
set CLANG=C:\Users\二手精品莆田鞋\.renesas\platform\arm-llvm\22.1.0\ATfE-22.1.0-Windows-x86_64\bin\clang.exe

if exist build\Debug rmdir /s /q build\Debug

"%CMAKE%" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_MAKE_PROGRAM="%NINJA%" -DCMAKE_C_COMPILER="%CLANG%" -S . -B build\Debug -G Ninja
if errorlevel 1 (
  echo [CONFIG FAILED]
  exit /b 1
)

"%CMAKE%" --build build\Debug
if errorlevel 1 (
  echo [BUILD FAILED]
  exit /b 1
)

echo [BUILD OK]
dir build\Debug\*.elf
