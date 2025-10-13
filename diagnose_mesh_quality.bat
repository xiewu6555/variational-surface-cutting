@echo off
REM Variational Surface Cutting - Mesh Quality Diagnostic Tool
REM Version: 2.0
REM Purpose: 快速诊断网格是否满足"no interior vertices"的要求

setlocal enabledelayedexpansion

echo ========================================
echo Variational Surface Cutting
echo Mesh Quality Diagnostic Tool v2.0
echo ========================================
echo.

echo This diagnostic script helps identify why "no interior vertices" error occurs.
echo.
echo CRITICAL UNDERSTANDING:
echo   - Yamabe equation requires patches with INTERIOR vertices
echo   - Normal Clustering creates 6-8 patches from your mesh
echo   - Each patch needs at least 30 interior vertices to solve
echo   - Low resolution mesh = some patches have 0 interior vertices = ERROR
echo.
echo SOLUTION HIERARCHY:
echo   Level 1: Increase mesh resolution (Target Edge Length = 0.012-0.018)
echo   Level 2: Use smaller Target Edge Length if Level 1 fails
echo   Level 3: Simplify model geometry (remove sharp features)
echo.
echo ========================================
echo INTERACTIVE DIAGNOSTIC
echo ========================================
echo.

REM 询问用户当前状态
echo What is your current situation?
echo [1] I haven't tried remeshing yet
echo [2] I remeshed but still got "no interior vertices" error
echo [3] I want to understand the optimal parameters
echo.
set /p "CHOICE=Enter choice (1/2/3): "
echo.

if "%CHOICE%"=="1" goto :FirstTime
if "%CHOICE%"=="2" goto :StillFailing
if "%CHOICE%"=="3" goto :Parameters
goto :Usage

:FirstTime
echo ========================================
echo First-Time User Guide
echo ========================================
echo.
echo STEP-BY-STEP INSTRUCTIONS:
echo.
echo Step 1: Open ImGui GUI
echo   - Run: build\STM-Debug\bin\Debug\SurfaceTextureMapping_imgui_gui.exe
echo.
echo Step 2: Load Mesh
echo   - Input File: data/spot.obj (or your mesh)
echo   - Click "Load"
echo.
echo Step 3: Configure Remeshing (CRITICAL!)
echo   [Control Panel - Mesh Processing section]
echo   - Check "Enable Remeshing" checkbox
echo   - Set "Target Edge Length" = 0.015
echo     (This is the SWEET SPOT value)
echo   - Set "Remesh Iterations" = 8
echo   - Check "Protect Boundary" if you want to preserve original boundary
echo.
echo Step 4: Execute Remeshing
echo   - Click "Step 1: Process Mesh"
echo   - WAIT for completion (may take 30-60 seconds)
echo   - Check "Statistics" panel:
echo     * Vertices should be 4000-8000
echo     * Avg Edge Length should be ~0.015
echo     * Max/Min ratio should be < 10.0
echo.
echo Step 5: Execute Cutting
echo   - Click "Step 2: Compute Cuts"
echo   - If no error, SUCCESS!
echo   - If "no interior vertices" error, see Choice [2]
echo.
echo ========================================
pause
goto :end

:StillFailing
echo ========================================
echo Troubleshooting "No Interior Vertices"
echo ========================================
echo.
echo You've remeshed but still getting the error. Let's diagnose:
echo.
echo QUESTION 1: What is your FINAL vertex count?
echo   (Check "Statistics" panel after Step 1)
echo.
set /p "VCOUNT=Enter vertex count: "
echo.

if %VCOUNT% LSS 3000 (
    echo [DIAGNOSIS] Vertex count TOO LOW!
    echo   Current: %VCOUNT%
    echo   Minimum: 4000 (recommended)
    echo   Target: 5000-7000 (optimal)
    echo.
    echo SOLUTION:
    echo   - Set Target Edge Length = 0.012 (smaller = more vertices)
    echo   - Or set = 0.010 for maximum density
    echo   - Retry Step 1 and Step 2
    echo.
) else if %VCOUNT% LSS 5000 (
    echo [DIAGNOSIS] Vertex count MARGINAL
    echo   Current: %VCOUNT%
    echo   This is borderline - may work, may fail
    echo.
    echo SOLUTION:
    echo   - Reduce Target Edge Length to 0.012
    echo   - This will increase vertices to ~7k
    echo   - Higher success rate guaranteed
    echo.
) else if %VCOUNT% GTR 15000 (
    echo [DIAGNOSIS] Vertex count TOO HIGH!
    echo   Current: %VCOUNT%
    echo   Performance will be very slow
    echo.
    echo SOLUTION:
    echo   - Increase Target Edge Length to 0.015
    echo   - This will reduce vertices to ~6k
    echo   - Still sufficient, much faster
    echo.
) else (
    echo [DIAGNOSIS] Vertex count is GOOD
    echo   Current: %VCOUNT%
    echo   This should work...
    echo.
    echo POSSIBLE CAUSES:
    echo   1. Model has very sharp features (ears, nose tips)
    echo   2. Some patches still lack interior vertices despite good total
    echo.
    echo ADVANCED SOLUTIONS:
    echo   - Try even smaller Target Edge Length (0.010)
    echo   - Or simplify model geometry (smooth sharp features in Blender/MeshLab)
    echo   - Or modify Normal Clustering algorithm (developer level)
    echo.
)

echo ========================================
echo.
echo QUESTION 2: What is your Edge Length Ratio?
echo   (Check log for "Edge length ratio (max/min): X.X")
echo.
set /p "RATIO=Enter ratio: "
echo.

if %RATIO% GTR 10 (
    echo [DIAGNOSIS] Edge ratio TOO LARGE!
    echo   Current: %RATIO%
    echo   Maximum: 10.0
    echo.
    echo This indicates non-uniform mesh, which causes:
    echo   - Ill-conditioned matrices
    echo   - Numerical instability
    echo   - Patches with no interior vertices
    echo.
    echo SOLUTION:
    echo   - Increase "Remesh Iterations" to 10-15
    echo   - This will better equalize edge lengths
    echo   - Retry Step 1 and Step 2
    echo.
) else (
    echo [DIAGNOSIS] Edge ratio is GOOD
    echo   Current: %RATIO%
    echo   Mesh uniformity is acceptable
    echo.
)

echo ========================================
echo.
echo NEXT STEPS:
echo   1. Apply the solutions above
echo   2. Retry the full pipeline
echo   3. If still failing, see FIX_NO_INTERIOR_VERTICES.md for advanced options
echo.
pause
goto :end

:Parameters
echo ========================================
echo Optimal Parameters Guide
echo ========================================
echo.
echo TARGET EDGE LENGTH RECOMMENDATIONS:
echo.
echo   Value    Vertices  Quality   Speed    Use Case
echo   -------  --------  --------  -------  --------------------------
echo   0.025    ~2k       Poor      Fast     Testing only
echo   0.020    ~3k       Low       Fast     Simple geometry
echo   0.018    ~4k       Medium    Good     Conservative choice
echo   0.015    ~5.5k     High      Good     RECOMMENDED (sweet spot)
echo   0.012    ~7k       Very High Medium   Complex geometry
echo   0.010    ~10k      Maximum   Slow     Last resort
echo.
echo REMESH ITERATIONS:
echo   5-7:  Good for most models
echo   8-10: Better uniformity (recommended)
echo   10+:  Maximum quality (slower)
echo.
echo MODEL-SPECIFIC RECOMMENDATIONS:
echo   - Spot (dog): 0.015, 8 iterations
echo   - Simple sphere: 0.020, 5 iterations
echo   - Dragon (complex): 0.012, 10 iterations
echo   - Stanford bunny: 0.015, 8 iterations
echo.
echo PERFORMANCE ESTIMATES (Spot model):
echo   4k vertices:  Step1=20s, Step2=15s, Total~40s
echo   6k vertices:  Step1=35s, Step2=25s, Total~70s
echo   10k vertices: Step1=60s, Step2=45s, Total~120s
echo.
echo ========================================
pause
goto :end

:Usage
echo ERROR: Invalid choice. Please run again and select 1, 2, or 3.
pause
goto :end

:end
echo.
echo ========================================
echo For detailed technical documentation:
echo   - FIX_NO_INTERIOR_VERTICES.md (user guide)
echo   - ANALYSIS_no_interior_vertices_deep_dive.md (theory)
echo ========================================
echo.
endlocal
