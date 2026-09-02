<#
.SYNOPSIS
    Inventories MSFS 2020 AI-traffic packages (AIG / FSLTL / FSTraffic) and reports
    their texture footprint, so you can tell whether shrinking textures is worth it.

.DESCRIPTION
    Read-only. Touches nothing. For every DDS file in the matched packages it reads
    the 148-byte DDS header (dimensions, mip count, DXGI/FourCC format) and rolls the
    results up per package, per resolution, and per texture folder.

    It also projects how many bytes you would save by capping albedo textures at
    -TargetAlbedo and everything else (normal / composite / mask / lightmap) at
    -TargetOther, which is the number that decides whether the re-encode below is
    worth several hours of texconv time.

.EXAMPLE
    .\Scan-MsfsTraffic.ps1

.EXAMPLE
    .\Scan-MsfsTraffic.ps1 -CommunityPath 'D:\MSFS\Community' -TargetAlbedo 1024 -TargetOther 512
#>
#Requires -Version 5.1
[CmdletBinding()]
param(
    # Community folder(s). Auto-detected from UserCfg.opt when omitted.
    [string[]]$CommunityPath,

    # Package-name substrings to treat as AI traffic.
    [string[]]$Include = @('aig', 'fsltl', 'fstraffic', 'ai-traffic', 'aitraffic'),

    # Proposed cap for colour/albedo textures.
    [int]$TargetAlbedo = 1024,

    # Proposed cap for normal / composite / mask / lightmap textures.
    [int]$TargetOther = 512,

    # Per-file CSV. Set to '' to skip (the file can run to tens of thousands of rows).
    [string]$CsvPath = (Join-Path (Get-Location).Path 'msfs-traffic-scan.csv'),

    # How many of the heaviest texture folders to list.
    [int]$Top = 20
)

$ErrorActionPreference = 'Stop'

$DxgiNames = @{
    10 = 'R16G16B16A16_FLOAT'; 28 = 'R8G8B8A8_UNORM'; 29 = 'R8G8B8A8_UNORM_SRGB'
    49 = 'R8G8_UNORM';         61 = 'R8_UNORM'
    71 = 'BC1_UNORM';          72 = 'BC1_UNORM_SRGB'
    74 = 'BC2_UNORM';          75 = 'BC2_UNORM_SRGB'
    77 = 'BC3_UNORM';          78 = 'BC3_UNORM_SRGB'
    80 = 'BC4_UNORM';          81 = 'BC4_SNORM'
    83 = 'BC5_UNORM';          84 = 'BC5_SNORM'
    87 = 'B8G8R8A8_UNORM';     91 = 'B8G8R8A8_UNORM_SRGB'
    95 = 'BC6H_UF16';          96 = 'BC6H_SF16'
    98 = 'BC7_UNORM';          99 = 'BC7_UNORM_SRGB'
}

$FourCCNames = @{
    'DXT1' = 'BC1_UNORM'; 'DXT3' = 'BC2_UNORM'; 'DXT5' = 'BC3_UNORM'
    'ATI1' = 'BC4_UNORM'; 'BC4U' = 'BC4_UNORM'; 'BC4S' = 'BC4_SNORM'
    'ATI2' = 'BC5_UNORM'; 'BC5U' = 'BC5_UNORM'; 'BC5S' = 'BC5_SNORM'
}

# Anything matching this is a data texture the eye cannot resolve on distant AI.
$NonAlbedoPattern = '_(comp|norm|nrm|bump|mask|metal|rough|occl|ao|spec|emis|lm|light|detail|wfdt|blend|opacity)\b'

function Resolve-CommunityPath {
    $opts = @(
        (Join-Path $env:LOCALAPPDATA 'Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalCache\UserCfg.opt')
        (Join-Path $env:APPDATA     'Microsoft Flight Simulator\UserCfg.opt')
        (Join-Path $env:LOCALAPPDATA 'MSFSPackages\UserCfg.opt')
    )
    $found = New-Object System.Collections.Generic.List[string]
    foreach ($opt in $opts) {
        if (-not (Test-Path -LiteralPath $opt)) { continue }
        foreach ($line in (Get-Content -LiteralPath $opt -ErrorAction SilentlyContinue)) {
            if ($line -match '^\s*InstalledPackagesPath\s+"(.+)"\s*$') {
                $c = Join-Path $Matches[1] 'Community'
                if ((Test-Path -LiteralPath $c) -and -not $found.Contains($c)) { $found.Add($c) }
            }
        }
    }
    return $found
}

function Read-DdsInfo {
    param([string]$Path)
    $fs = $null
    try {
        $fs = [System.IO.File]::Open($Path, [System.IO.FileMode]::Open,
                                     [System.IO.FileAccess]::Read,
                                     [System.IO.FileShare]::ReadWrite)
        $buf = New-Object byte[] 148
        $read = 0
        while ($read -lt 148) {
            $n = $fs.Read($buf, $read, 148 - $read)
            if ($n -le 0) { break }
            $read += $n
        }
        if ($read -lt 128) { return $null }
        if ([System.Text.Encoding]::ASCII.GetString($buf, 0, 4) -ne 'DDS ') { return $null }

        $height = [BitConverter]::ToUInt32($buf, 12)
        $width  = [BitConverter]::ToUInt32($buf, 16)
        $mips   = [BitConverter]::ToUInt32($buf, 28)
        $fourCC = [System.Text.Encoding]::ASCII.GetString($buf, 84, 4)

        $format = $null
        if ($fourCC -eq 'DX10') {
            if ($read -ge 148) {
                $dxgi = [BitConverter]::ToUInt32($buf, 128)
                $format = $DxgiNames[[int]$dxgi]
                if (-not $format) { $format = "DXGI_$dxgi" }
            } else {
                $format = 'DX10_TRUNCATED'
            }
        } elseif ($FourCCNames.ContainsKey($fourCC)) {
            $format = $FourCCNames[$fourCC]
        } else {
            $format = 'UNCOMPRESSED'
        }

        return [pscustomobject]@{
            Width = [int]$width; Height = [int]$height
            Mips = [int]$mips;   Format = $format
        }
    } catch {
        return $null
    } finally {
        if ($fs) { $fs.Dispose() }
    }
}

function Get-TargetDimensions {
    param([int]$Width, [int]$Height, [int]$Cap)
    $long = [Math]::Max($Width, $Height)
    if ($long -le $Cap) { return @($Width, $Height) }
    $scale = 1
    while (($long / $scale) -gt $Cap) { $scale *= 2 }
    # Block-compressed formats need at least one 4x4 block per side.
    $w = [Math]::Max(4, [int][Math]::Floor($Width / $scale))
    $h = [Math]::Max(4, [int][Math]::Floor($Height / $scale))
    return @($w, $h)
}

function Format-Size {
    param([double]$Bytes)
    if ($Bytes -ge 1GB) { return ('{0,8:N2} GB' -f ($Bytes / 1GB)) }
    if ($Bytes -ge 1MB) { return ('{0,8:N1} MB' -f ($Bytes / 1MB)) }
    return ('{0,8:N0} KB' -f ($Bytes / 1KB))
}

# ---------------------------------------------------------------- discovery --

if (-not $CommunityPath -or $CommunityPath.Count -eq 0) {
    $CommunityPath = Resolve-CommunityPath
}
if (-not $CommunityPath -or $CommunityPath.Count -eq 0) {
    throw "Could not auto-detect a Community folder. Pass -CommunityPath 'X:\...\Community'."
}

Write-Host ''
Write-Host 'MSFS 2020 AI-traffic texture scan' -ForegroundColor Cyan
Write-Host ('=' * 78) -ForegroundColor DarkGray
foreach ($c in $CommunityPath) { Write-Host "Community : $c" }

$packages = foreach ($c in $CommunityPath) {
    Get-ChildItem -LiteralPath $c -Directory -ErrorAction SilentlyContinue | Where-Object {
        $name = $_.Name
        ($Include | Where-Object { $name -like "*$_*" }).Count -gt 0
    }
}
$packages = @($packages)

if ($packages.Count -eq 0) {
    Write-Warning "No packages matched: $($Include -join ', ')"
    return
}

Write-Host "Packages  : $($packages.Count) matched"
Write-Host ''

# ----------------------------------------------------------------- scanning --

$rows      = New-Object System.Collections.Generic.List[object]
$pkgStats  = @{}
$folderAgg = @{}
$resAgg    = @{}
$pkgIndex  = 0

foreach ($pkg in $packages) {
    $pkgIndex++
    Write-Progress -Activity 'Scanning packages' -Status $pkg.Name `
                   -PercentComplete (100 * $pkgIndex / $packages.Count)

    $stat = [pscustomobject]@{
        Name          = $pkg.Name
        Path          = $pkg.FullName
        TotalBytes    = [long]0
        DdsBytes      = [long]0
        DdsCount      = 0
        OversizeCount = 0
        ProjectedDds  = [double]0
        HasLayout     = (Test-Path -LiteralPath (Join-Path $pkg.FullName 'layout.json'))
    }

    $files = Get-ChildItem -LiteralPath $pkg.FullName -File -Recurse -Force -ErrorAction SilentlyContinue
    foreach ($f in $files) {
        $stat.TotalBytes += $f.Length
        if ($f.Extension -notmatch '^\.dds$') { continue }

        $info = Read-DdsInfo -Path $f.FullName
        if (-not $info) { continue }

        $stat.DdsCount++
        $stat.DdsBytes += $f.Length

        $isAlbedo = -not ($f.Name -match $NonAlbedoPattern)
        $cap      = if ($isAlbedo) { $TargetAlbedo } else { $TargetOther }
        $dims     = Get-TargetDimensions -Width $info.Width -Height $info.Height -Cap $cap
        $tw, $th  = $dims[0], $dims[1]

        $projected = $f.Length
        if ($tw -ne $info.Width -or $th -ne $info.Height) {
            $stat.OversizeCount++
            $ratio = ([double]$tw * $th) / ([double]$info.Width * $info.Height)
            $projected = $f.Length * $ratio
        }
        $stat.ProjectedDds += $projected

        $resKey = '{0}x{1}' -f $info.Width, $info.Height
        if (-not $resAgg.ContainsKey($resKey)) {
            $resAgg[$resKey] = [pscustomobject]@{ Res = $resKey; Long = [Math]::Max($info.Width, $info.Height); Count = 0; Bytes = [long]0 }
        }
        $resAgg[$resKey].Count++
        $resAgg[$resKey].Bytes += $f.Length

        $dir = $f.DirectoryName
        if (-not $folderAgg.ContainsKey($dir)) {
            $folderAgg[$dir] = [pscustomobject]@{ Folder = $dir; Count = 0; Bytes = [long]0 }
        }
        $folderAgg[$dir].Count++
        $folderAgg[$dir].Bytes += $f.Length

        if ($CsvPath) {
            $rows.Add([pscustomobject]@{
                Package        = $pkg.Name
                RelativePath   = $f.FullName.Substring($pkg.FullName.Length).TrimStart('\')
                Width          = $info.Width
                Height         = $info.Height
                Mips           = $info.Mips
                Format         = $info.Format
                Class          = if ($isAlbedo) { 'albedo' } else { 'data' }
                Bytes          = $f.Length
                TargetWidth    = $tw
                TargetHeight   = $th
                ProjectedBytes = [long]$projected
            })
        }
    }

    $pkgStats[$pkg.FullName] = $stat
}
Write-Progress -Activity 'Scanning packages' -Completed

# ------------------------------------------------------------------ reports --

$all = $pkgStats.Values | Sort-Object -Property DdsBytes -Descending

Write-Host 'Per package' -ForegroundColor Cyan
Write-Host ('-' * 78) -ForegroundColor DarkGray
$fmt = '{0,-44} {1,7} {2,12} {3,12}'
Write-Host ($fmt -f 'Package', 'DDS', 'On disk', 'Texture MB')
foreach ($s in $all) {
    Write-Host ($fmt -f `
        $(if ($s.Name.Length -gt 44) { $s.Name.Substring(0, 41) + '...' } else { $s.Name }),
        $s.DdsCount,
        (Format-Size $s.TotalBytes).Trim(),
        ('{0:N1}' -f ($s.DdsBytes / 1MB)))
    if (-not $s.HasLayout) { Write-Host "    ! no layout.json in this package" -ForegroundColor Yellow }
}

$totalAll  = ($all | Measure-Object -Property TotalBytes   -Sum).Sum
$totalDds  = ($all | Measure-Object -Property DdsBytes     -Sum).Sum
$totalProj = ($all | Measure-Object -Property ProjectedDds -Sum).Sum
$totalCnt  = ($all | Measure-Object -Property DdsCount     -Sum).Sum
$totalOver = ($all | Measure-Object -Property OversizeCount -Sum).Sum

Write-Host ''
Write-Host 'Resolution histogram (all matched packages)' -ForegroundColor Cyan
Write-Host ('-' * 78) -ForegroundColor DarkGray
Write-Host ('{0,-14} {1,9} {2,14}' -f 'Resolution', 'Files', 'Bytes')
foreach ($r in ($resAgg.Values | Sort-Object -Property Long, Res -Descending)) {
    Write-Host ('{0,-14} {1,9:N0} {2,14}' -f $r.Res, $r.Count, (Format-Size $r.Bytes).Trim())
}

Write-Host ''
Write-Host "Heaviest texture folders (top $Top)" -ForegroundColor Cyan
Write-Host ('-' * 78) -ForegroundColor DarkGray
foreach ($f in ($folderAgg.Values | Sort-Object -Property Bytes -Descending | Select-Object -First $Top)) {
    Write-Host ('{0,10}  {1,5} files  {2}' -f (Format-Size $f.Bytes).Trim(), $f.Count, $f.Folder)
}

Write-Host ''
Write-Host 'Summary' -ForegroundColor Cyan
Write-Host ('-' * 78) -ForegroundColor DarkGray
Write-Host ("  Packages scanned        : {0}" -f $all.Count)
Write-Host ("  DDS textures            : {0:N0}" -f $totalCnt)
Write-Host ("  Package size on disk    : {0}" -f (Format-Size $totalAll).Trim())
Write-Host ("  Texture bytes           : {0}" -f (Format-Size $totalDds).Trim())
Write-Host ("  Above the proposed caps : {0:N0} files (albedo>{1}, data>{2})" -f $totalOver, $TargetAlbedo, $TargetOther)
Write-Host ("  After re-encode         : {0}" -f (Format-Size $totalProj).Trim())
if ($totalDds -gt 0) {
    $saved = $totalDds - $totalProj
    Write-Host ("  Saved                   : {0}  ({1:N0}%)" -f (Format-Size $saved).Trim(), (100 * $saved / $totalDds)) -ForegroundColor Green
}

# Overlapping model libraries cost VRAM and load time for nothing.
$families = @{}
foreach ($s in $all) {
    $fam = switch -Regex ($s.Name) {
        'fsltl'     { 'FSLTL'; break }
        'fstraffic' { 'FSTraffic'; break }
        'aig'       { 'AIG'; break }
        default     { 'Other' }
    }
    if (-not $families.ContainsKey($fam)) { $families[$fam] = [long]0 }
    $families[$fam] += $s.TotalBytes
}
if ($families.Keys.Count -gt 1) {
    Write-Host ''
    Write-Host 'Overlapping model libraries are active:' -ForegroundColor Yellow
    foreach ($k in ($families.Keys | Sort-Object)) {
        Write-Host ("    {0,-12} {1}" -f $k, (Format-Size $families[$k]).Trim()) -ForegroundColor Yellow
    }
    Write-Host '    Your injector matches against one of these. Disabling the rest is' -ForegroundColor Yellow
    Write-Host '    free performance and costs no visual quality. Do this before re-encoding.' -ForegroundColor Yellow
}

if ($CsvPath) {
    $rows | Export-Csv -LiteralPath $CsvPath -NoTypeInformation -Encoding UTF8
    Write-Host ''
    Write-Host "Per-file CSV written to $CsvPath ($($rows.Count) rows)" -ForegroundColor DarkGray
}
Write-Host ''
