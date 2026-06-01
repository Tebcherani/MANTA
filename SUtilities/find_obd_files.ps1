param(
    [string]$Drive = "K:\"
)

# Normalize drive letter
$driveLetter = $Drive.Substring(0,1).ToUpper()

$outfile = "ODB_Files_${driveLetter}.csv"

Write-Host ""
Write-Host "Scanning $Drive ..."
Write-Host ""

Get-ChildItem -Path $Drive -Recurse -File -Filter *.obd -ErrorAction SilentlyContinue |
    Select-Object FullName,
                  Name,
                  DirectoryName,
                  Length,
                  LastWriteTime |
    Sort-Object FullName |
    Export-Csv $outfile -NoTypeInformation

$count = (Import-Csv $outfile).Count

Write-Host ""
Write-Host "Found $count ODB files"
Write-Host "Inventory written to $outfile"