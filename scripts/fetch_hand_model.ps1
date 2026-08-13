param(
    [string]$Destination = "assets/models/hand_landmarker.task"
)

$ErrorActionPreference = "Stop"
$ExpectedSha256 = "FBC2A30080C3C557093B5DDFC334698132EB341044CCEE322CCF8BCF3607CDE1"
$ModelUrl = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$Target = Join-Path $ProjectRoot $Destination

New-Item -ItemType Directory -Force (Split-Path -Parent $Target) | Out-Null
Invoke-WebRequest -UseBasicParsing $ModelUrl -OutFile $Target
$ActualSha256 = (Get-FileHash $Target -Algorithm SHA256).Hash
if ($ActualSha256 -ne $ExpectedSha256) {
    throw "Hand Landmarker model checksum mismatch. Expected $ExpectedSha256 but found $ActualSha256."
}
Write-Host "Verified Hand Landmarker model: $Target"
