<#
.SYNOPSIS
Open a completed pothole run in VS Visualizer.

.DESCRIPTION
Two things made this script show a vehicle floating in mid-air over an empty
background, and both are fixed here.

1. It pointed at `runs\hd_utility_ddev_single_wheel_deep_pothole\native`, a
   history recorded *before* the road path was lengthened. That history's merged
   parameter file still declares `SPATH_START 0 / SEGMENT_LENGTH 40` while the
   vehicle starts at station 100 and every surface strip is declared at stations
   95-140 m, so no road geometry existed anywhere under the vehicle. It now
   defaults to the current control object (the corner module) and refuses to open
   a history whose road does not cover the run.

2. It launched the Visualizer with working directory `TruckSim2019.0_Data`.
   Relative `MTL_FILE` and 3D-shape paths resolve against the *resource root*,
   `TruckSim2019.0_Prog\Resources`; `_Data\Animator` contains neither
   `Road_Materials\road.mtl` nor `3D_Shape_Files\Trucks`, so the road surface and
   the truck meshes could not resolve and the scene rendered empty.

The history is staged into an ASCII-only directory because VS Visualizer mangles
non-ASCII command-line arguments (and this project's path contains Chinese
characters) into underscores.

.PARAMETER Model
`corner_module` (default) opens the current control object's run;
`truck` opens the retained solid-axle HD utility run.

.EXAMPLE
$env:PYTHONPATH='src'; python scripts\run_expert_pothole.py
powershell -File scripts\open_pothole_visualizer.ps1
#>
[CmdletBinding()]
param(
    [ValidateSet('corner_module', 'truck')]
    [string]$Model = 'corner_module',
    [int]$Width = 1280,
    [int]$Height = 720,
    [double]$CameraTimeS = 0.0
)

$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path

# The resource root, NOT `..._Data`: relative MTL_FILE / 3D shape paths resolve here.
$resourceRoot = 'F:\TruckSim2019\TruckSim2019.0_Prog\Resources'
$visualizer = 'F:\TruckSim2019\TruckSim2019.0_Prog\Programs\VsVisualizer\VsVisualizer.exe'
$stageRoot = 'F:\DDEV_TruckSim_Visualizer'
$historyName = 'single_wheel_deep_pothole'

switch ($Model) {
    'corner_module' {
        $runDir = Join-Path $projectRoot 'runs\corner_module_expert_pothole'
        $description = 'Corner Module DDEV (control object)'
    }
    'truck' {
        $runDir = Join-Path $projectRoot 'runs\hd_utility_ddev_expert_pothole'
        $description = 'HD Utility DDEV solid-axle truck (retained comparison)'
    }
}

$nativeDir = Join-Path $runDir 'native'
$history = Join-Path $nativeDir "$historyName.vs"
$binary = Join-Path $nativeDir "$historyName.vsb"
$mergedPar = Join-Path $nativeDir "${historyName}_all.par"

$required = @($history, $binary, $mergedPar, $visualizer, $resourceRoot)
foreach ($path in $required) {
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Missing required Visualizer input: $path"
    }
}

# Guard against exactly the mistake that produced the 'floating vehicle' report:
# opening a history recorded against a road too short to reach the vehicle.
$parText = Get-Content -LiteralPath $mergedPar -Raw
$segment = [regex]::Match($parText, '(?m)^\s*SEGMENT_LENGTH\s+([-0-9.eE]+)')
$start = [regex]::Match($parText, '(?m)^\s*SPATH_START\s+([-0-9.eE]+)')
$sstart = [regex]::Match($parText, '(?m)^\s*SSTART\s+([-0-9.eE]+)')
if ($segment.Success -and $start.Success -and $sstart.Success) {
    $roadEnd = [double]$start.Groups[1].Value + [double]$segment.Groups[1].Value
    $vehicleStart = [double]$sstart.Groups[1].Value
    if ($vehicleStart -ge $roadEnd) {
        throw ("History $history was recorded against a road that ends at station " +
               "$roadEnd m, but the vehicle starts at $vehicleStart m. Nothing would " +
               "render under it. Re-run scripts\run_expert_pothole.py to record a " +
               "history against the current model, then try again.")
    }
    Write-Host ("Road path covers 0-{0} m; vehicle starts at {1} m." -f $roadEnd, $vehicleStart)
}

New-Item -ItemType Directory -Path $stageRoot -Force | Out-Null
Copy-Item -LiteralPath $history -Destination (Join-Path $stageRoot "$historyName.vs") -Force
Copy-Item -LiteralPath $binary -Destination (Join-Path $stageRoot "$historyName.vsb") -Force
Copy-Item -LiteralPath $mergedPar -Destination (Join-Path $stageRoot "${historyName}_all.par") -Force

$animatorPath = Join-Path $stageRoot 'animator.par'
$animator = @"
PARSFILE
SET_RUN_SLOT 0
DATASET $stageRoot\$historyName.vs
PARSFILE $stageRoot\${historyName}_all.par
END
"@
Set-Content -LiteralPath $animatorPath -Value $animator -Encoding Ascii

Start-Process -FilePath $visualizer `
    -WorkingDirectory $resourceRoot `
    -ArgumentList @('-i', 'DDEV_Pothole', '-fs', "$CameraTimeS", '-res', "$Width", "$Height", '-tb', '1', $animatorPath)

Write-Host "VS Visualizer opened on: $description"
Write-Host "  working directory : $resourceRoot"
Write-Host "  history           : $history"
Write-Host 'Use File > Export Video, set 0-9 s, then choose an AVI compressor.'
