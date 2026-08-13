param(
    [Parameter(Mandatory = $true)]
    [string]$MediaPipeSource,
    [string]$OutputDirectory = "build/windows-debug"
)

$ErrorActionPreference = "Stop"
$ExpectedCommit = "f8ef212d5c962c0e853db7e59d217056b187084b"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$ResolvedMediaPipe = (Resolve-Path $MediaPipeSource).Path
$BazelOutputRoot = "C:\bzl-air"

if (-not (Test-Path (Join-Path $ResolvedMediaPipe "WORKSPACE"))) {
    throw "MediaPipeSource must point to the root of the MediaPipe v0.10.35 repository."
}

$ActualCommit = (& git -C $ResolvedMediaPipe rev-parse HEAD).Trim()
if ($ActualCommit -ne $ExpectedCommit) {
    throw "Expected MediaPipe commit $ExpectedCommit but found $ActualCommit."
}

$BridgePackage = Join-Path $ResolvedMediaPipe "aerial_touch_bridge"
New-Item -ItemType Directory -Force $BridgePackage | Out-Null
Copy-Item (Join-Path $ProjectRoot "mediapipe_bridge/bridge.cc") (Join-Path $BridgePackage "bridge.cc") -Force
Copy-Item (Join-Path $ProjectRoot "mediapipe_bridge/hand_bridge_api.h") (Join-Path $BridgePackage "hand_bridge_api.h") -Force
Copy-Item (Join-Path $ProjectRoot "mediapipe_bridge/BUILD") (Join-Path $BridgePackage "BUILD") -Force

$CompatibilityPatches = @(
    (Join-Path $ProjectRoot "mediapipe_bridge/mediapipe_v0.10.35_windows.patch"),
    (Join-Path $ProjectRoot "mediapipe_bridge/mediapipe_v0.10.35_windows_api3.patch"),
    (Join-Path $ProjectRoot "mediapipe_bridge/mediapipe_v0.10.35_windows_visit.patch"),
    (Join-Path $ProjectRoot "mediapipe_bridge/mediapipe_v0.10.35_windows_node_name.patch")
)
Push-Location $ResolvedMediaPipe
try {
    foreach ($CompatibilityPatch in $CompatibilityPatches) {
        & git apply --reverse --check $CompatibilityPatch 2>$null
        if ($LASTEXITCODE -ne 0) {
            & git apply --check $CompatibilityPatch
            if ($LASTEXITCODE -ne 0) {
                throw "MediaPipe Windows compatibility patch no longer applies cleanly: $CompatibilityPatch"
            }
            & git apply $CompatibilityPatch
            if ($LASTEXITCODE -ne 0) {
                throw "Failed to apply the MediaPipe Windows compatibility patch: $CompatibilityPatch"
            }
        }
    }
}
finally {
    Pop-Location
}

$OpenCvRoot = Join-Path $ProjectRoot "build/windows-debug/vcpkg_installed/x64-windows"
if (-not (Test-Path (Join-Path $OpenCvRoot "include/opencv4/opencv2/core.hpp"))) {
    throw "The vcpkg OpenCV installation was not found at $OpenCvRoot. Configure the windows-debug CMake preset first."
}
$OpenCvRepository = Join-Path $ProjectRoot "build/mediapipe_opencv_repo"
New-Item -ItemType Directory -Force $OpenCvRepository | Out-Null
Copy-Item (Join-Path $ProjectRoot "mediapipe_bridge/opencv_vcpkg.BUILD") (Join-Path $OpenCvRepository "BUILD.bazel") -Force
Copy-Item (Join-Path $ProjectRoot "mediapipe_bridge/opencv_vcpkg.WORKSPACE") (Join-Path $OpenCvRepository "WORKSPACE") -Force
foreach ($Directory in @("include", "lib", "bin", "debug")) {
    $LinkPath = Join-Path $OpenCvRepository $Directory
    if (-not (Test-Path $LinkPath)) {
        New-Item -ItemType Junction -Path $LinkPath -Target (Join-Path $OpenCvRoot $Directory) | Out-Null
    }
}

if (-not $env:BAZEL_VC) {
    $env:BAZEL_VC = "C:\Program Files\Microsoft Visual Studio\18\Community\VC"
}
if (-not $env:BAZEL_SH) {
    foreach ($Candidate in @("C:\msys64\usr\bin\bash.exe", "C:\tools\msys64\usr\bin\bash.exe")) {
        if (Test-Path $Candidate) {
            $env:BAZEL_SH = $Candidate
            break
        }
    }
}
if (-not $env:BAZEL_SH) {
    throw "MSYS2 bash was not found. Install MSYS2 or set BAZEL_SH to usr/bin/bash.exe."
}
if (-not $env:HERMETIC_PYTHON_VERSION) {
    $env:HERMETIC_PYTHON_VERSION = "3.12"
}

Push-Location $ResolvedMediaPipe
try {
    & bazelisk --output_user_root=$BazelOutputRoot build -c opt --config=windows --repo_env=HERMETIC_PYTHON_VERSION=$env:HERMETIC_PYTHON_VERSION "--override_repository=windows_opencv=$OpenCvRepository" --define MEDIAPIPE_DISABLE_GPU=1 --copt=/DMEDIAPIPE_DISABLE_GPU=1 --conlyopt=/std:c11 --conlyopt=/experimental:c11atomics --cxxopt=/Zc:preprocessor --cxxopt=/utf-8 //aerial_touch_bridge:mediapipe_hand_bridge.dll
    if ($LASTEXITCODE -ne 0) {
        throw "Bazel bridge build failed with exit code $LASTEXITCODE."
    }
}
finally {
    Pop-Location
}

$BuiltDll = Join-Path $ResolvedMediaPipe "bazel-bin/aerial_touch_bridge/mediapipe_hand_bridge.dll"
if (-not (Test-Path $BuiltDll)) {
    throw "Bazel reported success but the expected DLL was not found: $BuiltDll"
}

$ResolvedOutput = Join-Path $ProjectRoot $OutputDirectory
New-Item -ItemType Directory -Force $ResolvedOutput | Out-Null
Copy-Item $BuiltDll (Join-Path $ResolvedOutput "mediapipe_hand_bridge.dll") -Force
foreach ($RuntimeDll in @("opencv_core4.dll", "opencv_imgproc4.dll", "z.dll")) {
    $RuntimePath = Join-Path $OpenCvRoot "bin/$RuntimeDll"
    if (-not (Test-Path $RuntimePath)) {
        throw "Required bridge runtime DLL was not found: $RuntimePath"
    }
    Copy-Item $RuntimePath (Join-Path $ResolvedOutput $RuntimeDll) -Force
}
Write-Host "MediaPipe bridge copied to $ResolvedOutput"
