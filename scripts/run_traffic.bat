@echo off
echo Starting CARLA Traffic Generator (50 vehicles, 20 walkers)...
python %CARLA_ROOT%/PythonAPI/examples/generate_traffic.py -n 50 -w 20
pause