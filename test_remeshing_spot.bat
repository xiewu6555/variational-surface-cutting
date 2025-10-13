@echo off
echo ========================================
echo Testing Remeshing Fix with Spot Model
echo ========================================
echo.
echo This script will:
echo 1. Run the ImGui GUI with spot.obj
echo 2. The GUI should automatically load with targetEdgeLength=0.005
echo 3. Click "Step 2: Compute Cuts" to trigger automatic remeshing
echo 4. Check the log for remeshing statistics
echo.
echo Expected Results:
echo - Remeshing should increase vertex count significantly
echo - Edge length ratio should drop below 10.0
echo - Variational Surface Cutting should succeed
echo.
pause

cd build\STM-Debug\bin\Debug
echo.
echo Starting GUI...
echo.
SurfaceTextureMapping_imgui_gui.exe

pause
