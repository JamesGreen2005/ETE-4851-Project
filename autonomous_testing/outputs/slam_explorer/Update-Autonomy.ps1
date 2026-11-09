param([string]$PiHost = '10.110.135.247', [string]$PiUser = 'swerve')
$ErrorActionPreference = 'Stop'
$archive = Join-Path (Split-Path $PSScriptRoot -Parent) 'slam_explorer_pi.zip'
if (-not (Test-Path -LiteralPath $archive)) { throw "Missing package: $archive" }
Write-Host 'Stop the old SLAM launch with Ctrl+C on the Pi before updating.'
Write-Host 'Uploading and installing SLAM Explorer 0.17. Enter the Pi password when asked.'
& scp $archive "${PiUser}@${PiHost}:~/slam_explorer_pi.zip"
if ($LASTEXITCODE -ne 0) { throw 'Upload failed.' }
& ssh -t "${PiUser}@${PiHost}" 'mkdir -p ~/slam_explorer_autonomy_update && python3 -m zipfile -e ~/slam_explorer_pi.zip ~/slam_explorer_autonomy_update && bash ~/slam_explorer_autonomy_update/install_pi.sh'
if ($LASTEXITCODE -ne 0) { throw 'Pi installation failed; check the message above.' }
Write-Host 'Installed. No camera, navigation, or motor process was started.'
Write-Host 'Follow FOOTPRINT_FIX.txt to start the cameras and dashboard.'
