$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$nativeDir = Join-Path $projectRoot 'runs\hd_utility_ddev_single_wheel_deep_pothole\native'
$visualizer = 'F:\TruckSim2019\TruckSim2019.0_Prog\Programs\VsVisualizer\VsVisualizer.exe'
$trucksimData = 'F:\TruckSim2019\TruckSim2019.0_Data'
$stageRoot = 'F:\DDEV_TruckSim_Visualizer'

$required = @(
    (Join-Path $nativeDir 'single_wheel_deep_pothole.vs'),
    (Join-Path $nativeDir 'single_wheel_deep_pothole.vsb'),
    (Join-Path $nativeDir 'single_wheel_deep_pothole_all.par'),
    $visualizer,
    $trucksimData
)
foreach ($path in $required) {
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Missing required Visualizer input: $path"
    }
}

New-Item -ItemType Directory -Path $stageRoot -Force | Out-Null
Copy-Item -LiteralPath $required[0] -Destination (Join-Path $stageRoot 'single_wheel_deep_pothole.vs') -Force
Copy-Item -LiteralPath $required[1] -Destination (Join-Path $stageRoot 'single_wheel_deep_pothole.vsb') -Force
Copy-Item -LiteralPath $required[2] -Destination (Join-Path $stageRoot 'single_wheel_deep_pothole_all.par') -Force

$animatorPath = Join-Path $stageRoot 'animator.par'
$animator = @"
PARSFILE
SET_RUN_SLOT 0
DATASET F:\DDEV_TruckSim_Visualizer\single_wheel_deep_pothole.vs
PARSFILE F:\DDEV_TruckSim_Visualizer\single_wheel_deep_pothole_all.par
END
"@
Set-Content -LiteralPath $animatorPath -Value $animator -Encoding Ascii

Start-Process -FilePath $visualizer `
    -WorkingDirectory $trucksimData `
    -ArgumentList @('-i', 'DDEV_Pothole', '-fs', '0', '-res', '1280', '720', '-tb', '1', $animatorPath)

Write-Host 'VS Visualizer opened with the completed DDEV pothole history.'
Write-Host 'Use File > Export Video, set 0-9 s, then choose an AVI compressor.'
