@echo off
echo Cleaning build artifacts...
for /d /r . %%d in (Debug) do @if exist "%%d" rd /s /q "%%d"
for /d /r . %%d in (Release) do @if exist "%%d" rd /s /q "%%d"
echo Project cleaned.
pause