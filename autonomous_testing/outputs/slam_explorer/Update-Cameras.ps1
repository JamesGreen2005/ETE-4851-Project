param([string]$PiHost = '192.168.1.160', [string]$PiUser = 'swerve')
$ErrorActionPreference = 'Stop'
$archive = Join-Path (Split-Path $PSScriptRoot -Parent) 'slam_explorer_pi.zip'
if (-not (Test-Path -LiteralPath $archive)) { throw "Missing package: $archive" }
Write-Host 'Stop the old SLAM launch with Ctrl+C on the Pi before updating.'
Write-Host 'Uploading and installing the camera update. Enter the Pi password when asked.'
& scp $archive "${PiUser}@${PiHost}:~/slam_explorer_pi.zip"
if ($LASTEXITCODE -ne 0) { throw 'Upload failed.' }
& ssh -t "${PiUser}@${PiHost}" 'mkdir -p ~/slam_explorer_camera_update && python3 -m zipfile -e ~/slam_explorer_pi.zip ~/slam_explorer_camera_update && bash ~/slam_explorer_camera_update/install_pi.sh'
if ($LASTEXITCODE -ne 0) { throw 'Pi installation failed; check the message above.' }
Write-Host 'Installed. No camera, navigation, or motor process was started.'
Write-Host 'Follow REAR_CAMERA_SETUP.txt to start the cameras and dashboard.'
