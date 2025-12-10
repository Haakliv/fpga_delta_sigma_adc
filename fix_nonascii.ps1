# Script to replace non-ASCII characters with ASCII equivalents
# Run this from the project root directory

$folders = @(
    "sources",
    "testbench",
    "scripts"
)

$replacements = @{
    # Arrow right
    [char]0x2192 = "->"
    # Plus-minus
    [char]0x00B1 = "+/-"
    # Less than or equal
    [char]0x2264 = "<="
    # Greater than or equal
    [char]0x2265 = ">="
    # Degree sign
    [char]0x00B0 = " deg"
    # Multiplication sign
    [char]0x00D7 = "x"
    # Division sign
    [char]0x00F7 = "/"
}

$fileCount = 0
$changeCount = 0

foreach ($folder in $folders) {
    $path = Join-Path $PSScriptRoot $folder
    if (Test-Path $path) {
        Write-Host "Processing folder: $folder" -ForegroundColor Cyan
        
        $files = Get-ChildItem -Path $path -Recurse -Include *.vhd,*.tcl,*.sdc,*.py -File
        
        foreach ($file in $files) {
            $content = Get-Content $file.FullName -Raw -Encoding UTF8
            $originalContent = $content
            $fileHadChanges = $false
            
            foreach ($key in $replacements.Keys) {
                if ($content.Contains($key)) {
                    $content = $content.Replace($key, $replacements[$key])
                    $fileHadChanges = $true
                }
            }
            
            if ($fileHadChanges) {
                Set-Content -Path $file.FullName -Value $content -Encoding UTF8 -NoNewline
                Write-Host "  Fixed: $($file.Name)" -ForegroundColor Green
                $fileCount++
                $changeCount++
            }
        }
    }
}

Write-Host "`nSummary:" -ForegroundColor Yellow
Write-Host "  Files modified: $fileCount" -ForegroundColor Yellow
Write-Host "  Total changes: $changeCount" -ForegroundColor Yellow
