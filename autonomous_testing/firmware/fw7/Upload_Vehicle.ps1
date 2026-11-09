# Upload this folder's compiled image directly, bypassing MATLAB model/path selection.
$ErrorActionPreference = 'Stop'
$imagePath = Join-Path $PSScriptRoot 'Vehicle_Drive.bin'
if (-not (Test-Path -LiteralPath $imagePath)) { throw 'Vehicle_Drive.bin is missing. Run build_vehicle in MATLAB first.' }
$cliCandidates = @(Get-ChildItem -LiteralPath 'C:\ST' -Filter 'STM32_Programmer_CLI.exe' -Recurse -ErrorAction SilentlyContinue)
if ($cliCandidates.Count -eq 0) { throw 'STM32CubeProgrammer was not found under C:\ST.' }
$cliPath = $cliCandidates[0].FullName
Write-Host 'Motor battery must be disconnected and BEC off. Connect Nucleo ST-LINK USB to this PC.'
$probes = & $cliPath -l stlink-only 2>&1
$probeText = $probes | Out-String
Write-Host $probeText
$probeCount = [regex]::Matches($probeText, 'ST-LINK SN\s*[:=]').Count
if ($probeCount -ne 1) { throw "Expected exactly one ST-LINK probe; found $probeCount. Connect only the intended Nucleo and retry." }
Write-Host 'Uploading encoder diagnostic FW=7: motor 1 encoder PC10/PC11, motor 2 encoder PC8/PC9, INA219 PB8/PB9.'
& $cliPath -c port=SWD mode=UR -d $imagePath 0x08000000 -v -rst
if ($LASTEXITCODE -ne 0) { throw 'Upload or verification failed. Keep motor power disconnected and read the programmer error above.' }
Write-Host 'UPLOAD AND FLASH VERIFICATION SUCCEEDED.' -ForegroundColor Green
Write-Host 'Reconnect ST-LINK USB to the Pi. Run the matching vehicle_drive dashboard. Board telemetry must include DRIVE=1 FW=7 INA=1. Use the matching ENCODER2-PC8-PC9-POSITIVE-8 dashboard.'
