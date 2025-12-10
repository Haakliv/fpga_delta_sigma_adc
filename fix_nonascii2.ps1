$folders = @("sources", "testbench", "scripts")
$extensions = @("*.vhd", "*.tcl", "*.sdc", "*.py")
$arrow = [char]0x2192
$plusminus = [char]0x00B1
$filesModified = 0
$totalChanges = 0

foreach ($folder in $folders) {
    $folderPath = "C:\Projects\agilex_5\refdes-agilex5\axe5000\accel_temp_refdes\$folder"
    Write-Host "Processing folder: $folder"
    
    foreach ($ext in $extensions) {
        $files = Get-ChildItem -Path $folderPath -Filter $ext -Recurse -File -ErrorAction SilentlyContinue
        foreach ($file in $files) {
            $content = Get-Content $file.FullName -Raw -Encoding UTF8
            $modified = $false
            
            if ($content -match $arrow) {
                $content = $content.Replace($arrow.ToString(), "->")
                $modified = $true
                $totalChanges++
            }
            
            if ($content -match $plusminus) {
                $content = $content.Replace($plusminus.ToString(), "+/-")
                $modified = $true
                $totalChanges++
            }
            
            if ($modified) {
                $content | Set-Content $file.FullName -Encoding UTF8 -NoNewline
                Write-Host "  Fixed: $($file.Name)"
                $filesModified++
            }
        }
    }
}

Write-Host "`nSummary:"
Write-Host "  Files modified: $filesModified"
Write-Host "  Total changes: $totalChanges"
