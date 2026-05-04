param([string]$Opts = 'all')
$sdata = 'D:\OneDriveJHU\OneDrive - Johns Hopkins\Matlab-JHU\AUV_Mission_Sim\AUV_Sim\Windows_Ody_C\SData'
$last  = Get-ChildItem "$sdata\*.obd" | Sort-Object LastWriteTime -Descending | Select-Object -First 1
if (-not $last) { Write-Error "No .obd files in $sdata"; exit 1 }
Write-Host "Reviewing $($last.Name)"
python -m ody_review.review $last.FullName --opts $Opts