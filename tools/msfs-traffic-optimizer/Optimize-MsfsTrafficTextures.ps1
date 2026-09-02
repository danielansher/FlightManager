<#
.SYNOPSIS
    Downscales the DDS textures inside MSFS 2020 AI-traffic packages (AIG / FSLTL /
    FSTraffic) in place, preserving each texture's original compression format.

.DESCRIPTION
    Dry run by default: it prints exactly what it would do and how much it would save.
    Nothing is written until you add -Apply.

    For each DDS it reads the header, picks a power-of-two downscale that brings the
    long edge to at most -MaxAlbedo (colour) or -MaxOther (normal / composite / mask /
    lightmap), then re-encodes with texconv into the SAME DXGI format with a full mip
    chain. The .DDS.json sidecars MSFS keeps next to each texture are left untouched.

    Because MSFS validates every file's size and date against the package's
    layout.json, layout.json MUST be regenerated afterwards. Pass -LayoutGenerator
    pointing at MSFSLayoutGenerator.exe and this script does it for you. Skip that
    step and the sim will, at best, ignore your changes and, at worst, CTD.

.NOTES
    Requires texconv.exe from Microsoft DirectXTex:
        https://github.com/microsoft/DirectXTex/releases
    Layout regeneration uses MSFSLayoutGenerator.exe by HughesMDflyer4:
        https://github.com/HughesMDflyer4/MSFSLayoutGenerator/releases

    BC7 re-encoding is slow on CPU. Add -UseGpu to encode BC7/BC6H on the GPU; it is
    roughly an order of magnitude faster and visually equivalent for AI traffic.

    AIG Manager / OCI and FSLTL updates overwrite package contents. Re-run this after
    any content update, or keep -BackupRoot so you can restore first.

.EXAMPLE
    # See what would change, change nothing.
    .\Optimize-MsfsTrafficTextures.ps1 -PackagePath 'D:\MSFS\Community\aig-*' -TexConv 'C:\tools\texconv.exe'

.EXAMPLE
    # Do it, with backups and layout regeneration.
    .\Optimize-MsfsTrafficTextures.ps1 `
        -PackagePath 'D:\MSFS\Community\aig-*','D:\MSFS\Community\fsltl-traffic-base' `
        -TexConv 'C:\tools\texconv.exe' `
        -LayoutGenerator 'C:\tools\MSFSLayoutGenerator.exe' `
        -BackupRoot 'E:\msfs-texture-backup' `
        -UseGpu -Apply

.EXAMPLE
    # Put everything back.
    .\Optimize-MsfsTrafficTextures.ps1 -BackupRoot 'E:\msfs-texture-backup' -Restore -Apply
#>
#Requires -Version 5.1
[CmdletBinding()]
param(
    # Package folders. Wildcards allowed.
    [string[]]$PackagePath,

    [string]$TexConv,

    [string]$LayoutGenerator,

    # Cap for colour/albedo textures.
    [int]$MaxAlbedo = 1024,

    # Cap for normal / composite / mask / lightmap textures.
    [int]$MaxOther = 512,

    # Where originals are copied before being overwritten. Strongly recommended.
    [string]$BackupRoot,

    # Encode BC7/BC6H on the GPU instead of the CPU.
    [switch]$UseGpu,

    # Copy everything in -BackupRoot back over the live packages, then exit.
    [switch]$Restore,

    # Without this the script only reports.
    [switch]$Apply
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
$NonAlbedoPattern = '_(comp|norm|nrm|bump|mask|metal|rough|occl|ao|spec|emis|lm|light|detail|wfdt|blend|opacity)\b'

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

        $height = [int][BitConverter]::ToUInt32($buf, 12)
        $width  = [int][BitConverter]::ToUInt32($buf, 16)
        $fourCC = [System.Text.Encoding]::ASCII.GetString($buf, 84, 4)

        $format = $null
        if ($fourCC -eq 'DX10') {
            if ($read -lt 148) { return $null }
            $format = $DxgiNames[[int][BitConverter]::ToUInt32($buf, 128)]
        } elseif ($FourCCNames.ContainsKey($fourCC)) {
            $format = $FourCCNames[$fourCC]
        }
        # $format stays $null for cube maps, volume textures and uncompressed or
        # unrecognised layouts. Those are skipped rather than guessed at.
        if (-not $format) { return $null }

        return [pscustomobject]@{ Width = $width; Height = $height; Format = $format }
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
    return @([Math]::Max(4, [int][Math]::Floor($Width / $scale)),
             [Math]::Max(4, [int][Math]::Floor($Height / $scale)))
}

function Format-Size {
    param([double]$Bytes)
    if ($Bytes -ge 1GB) { return ('{0:N2} GB' -f ($Bytes / 1GB)) }
    if ($Bytes -ge 1MB) { return ('{0:N1} MB' -f ($Bytes / 1MB)) }
    return ('{0:N0} KB' -f ($Bytes / 1KB))
}

# ------------------------------------------------------------------ restore --

if ($Restore) {
    if (-not $BackupRoot -or -not (Test-Path -LiteralPath $BackupRoot)) {
        throw "-Restore needs a -BackupRoot that exists."
    }
    $manifestPath = Join-Path $BackupRoot 'restore-manifest.csv'
    if (-not (Test-Path -LiteralPath $manifestPath)) {
        throw "No restore-manifest.csv in $BackupRoot; cannot map backups to their original locations."
    }
    $manifest = Import-Csv -LiteralPath $manifestPath
    Write-Host "Restoring $($manifest.Count) files from $BackupRoot" -ForegroundColor Cyan
    $n = 0
    foreach ($m in $manifest) {
        if (-not (Test-Path -LiteralPath $m.BackupPath)) {
            Write-Warning "missing backup: $($m.BackupPath)"
            continue
        }
        if ($Apply) {
            Copy-Item -LiteralPath $m.BackupPath -Destination $m.OriginalPath -Force
        }
        $n++
    }
    Write-Host ("{0} {1} files" -f $(if ($Apply) { 'Restored' } else { 'Would restore' }), $n) -ForegroundColor Green
    if ($Apply -and $LayoutGenerator) {
        foreach ($pkg in ($manifest.Package | Sort-Object -Unique)) {
            $layout = Join-Path $pkg 'layout.json'
            if (Test-Path -LiteralPath $layout) {
                Write-Host "  regenerating $layout"
                & $LayoutGenerator $layout | Out-Null
            }
        }
    } elseif ($Apply) {
        Write-Warning "Regenerate each package's layout.json before launching MSFS."
    }
    return
}

# ------------------------------------------------------------------- checks --

if (-not $PackagePath -or $PackagePath.Count -eq 0) { throw "-PackagePath is required." }
if (-not $TexConv) { throw "-TexConv is required (path to texconv.exe)." }

$texconvCmd = Get-Command $TexConv -ErrorAction SilentlyContinue
if (-not $texconvCmd) { throw "texconv not found at '$TexConv'. Get it from https://github.com/microsoft/DirectXTex/releases" }
$TexConv = $texconvCmd.Source

if ($LayoutGenerator) {
    $lg = Get-Command $LayoutGenerator -ErrorAction SilentlyContinue
    if (-not $lg) { throw "MSFSLayoutGenerator not found at '$LayoutGenerator'." }
    $LayoutGenerator = $lg.Source
}

$packages = @()
foreach ($p in $PackagePath) {
    $packages += Get-Item -Path $p -ErrorAction SilentlyContinue | Where-Object { $_.PSIsContainer }
}
$packages = @($packages | Sort-Object -Property FullName -Unique)
if ($packages.Count -eq 0) { throw "No package folders matched -PackagePath." }

Write-Host ''
Write-Host $(if ($Apply) { 'APPLYING CHANGES' } else { 'DRY RUN - nothing will be written (add -Apply to commit)' }) `
           -ForegroundColor $(if ($Apply) { 'Yellow' } else { 'Cyan' })
Write-Host ("Packages: {0}   albedo cap {1}   data cap {2}" -f $packages.Count, $MaxAlbedo, $MaxOther)
if ($Apply -and -not $BackupRoot) {
    Write-Warning "No -BackupRoot given. Originals will be overwritten with no way back."
}
if ($Apply -and -not $LayoutGenerator) {
    Write-Warning "No -LayoutGenerator given. You MUST regenerate layout.json yourself before launching MSFS."
}
Write-Host ''

# -------------------------------------------------------------------- plan ---

$plan = New-Object System.Collections.Generic.List[object]
foreach ($pkg in $packages) {
    $files = Get-ChildItem -LiteralPath $pkg.FullName -File -Recurse -Force -Filter '*.dds' -ErrorAction SilentlyContinue
    foreach ($f in $files) {
        $info = Read-DdsInfo -Path $f.FullName
        if (-not $info) { continue }
        $cap  = if ($f.Name -match $NonAlbedoPattern) { $MaxOther } else { $MaxAlbedo }
        $dims = Get-TargetDimensions -Width $info.Width -Height $info.Height -Cap $cap
        if ($dims[0] -eq $info.Width -and $dims[1] -eq $info.Height) { continue }
        $plan.Add([pscustomobject]@{
            Package = $pkg.FullName
            File    = $f.FullName
            Folder  = $f.DirectoryName
            Stem    = [System.IO.Path]::GetFileNameWithoutExtension($f.Name)
            Bytes   = $f.Length
            Format  = $info.Format
            Width   = $dims[0]
            Height  = $dims[1]
            Projected = [long]($f.Length * (([double]$dims[0] * $dims[1]) / ([double]$info.Width * $info.Height)))
        })
    }
}

if ($plan.Count -eq 0) {
    Write-Host 'Nothing exceeds the caps. No work to do.' -ForegroundColor Green
    return
}

$before = ($plan | Measure-Object -Property Bytes -Sum).Sum
$after  = ($plan | Measure-Object -Property Projected -Sum).Sum
Write-Host ("{0:N0} textures to re-encode" -f $plan.Count)
Write-Host ("  before : {0}" -f (Format-Size $before))
Write-Host ("  after  : {0}" -f (Format-Size $after))
Write-Host ("  saved  : {0} ({1:N0}%)" -f (Format-Size ($before - $after)), (100 * ($before - $after) / $before)) -ForegroundColor Green
Write-Host ''

if (-not $Apply) {
    Write-Host 'Sample of planned changes:' -ForegroundColor DarkGray
    foreach ($p in ($plan | Sort-Object -Property Bytes -Descending | Select-Object -First 15)) {
        Write-Host ("  {0,9}  ->  {1}x{2} {3}  {4}" -f (Format-Size $p.Bytes), $p.Width, $p.Height, $p.Format, $p.File) -ForegroundColor DarkGray
    }
    Write-Host ''
    Write-Host 'Re-run with -Apply (and -BackupRoot, -LayoutGenerator) to commit.' -ForegroundColor Cyan
    return
}

# ------------------------------------------------------------------- apply ---

if ($BackupRoot) { New-Item -ItemType Directory -Path $BackupRoot -Force | Out-Null }
$manifest = New-Object System.Collections.Generic.List[object]
$tempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("msfstex_" + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $tempRoot -Force | Out-Null

# texconv writes output using the source file name, so batching is only safe within
# a single source folder. Group by folder + format + target size so one invocation
# can carry many files with identical arguments.
$groups = $plan | Group-Object -Property { '{0}|{1}|{2}|{3}' -f $_.Folder, $_.Format, $_.Width, $_.Height }
$done = 0; $failed = 0; $groupIndex = 0

foreach ($g in $groups) {
    $groupIndex++
    $first  = $g.Group[0]
    $outDir = Join-Path $tempRoot ([Guid]::NewGuid().ToString('N'))
    New-Item -ItemType Directory -Path $outDir -Force | Out-Null

    Write-Progress -Activity 'Re-encoding textures' `
                   -Status ("{0} ({1} files)" -f $first.Folder, $g.Count) `
                   -PercentComplete (100 * $groupIndex / $groups.Count)

    $tcArgs = @('-nologo', '-y', '-f', $first.Format,
              '-w', $first.Width, '-h', $first.Height,
              '-m', '0', '-o', $outDir)
    if ($first.Format -like '*_SRGB') { $tcArgs += '-srgb' }
    if ($UseGpu -and ($first.Format -like 'BC7*' -or $first.Format -like 'BC6H*')) { $tcArgs += @('-gpu', '0') }

    # Keep each command line comfortably under the ~32k limit.
    $batch = New-Object System.Collections.Generic.List[string]
    $len = 0
    $batches = New-Object System.Collections.Generic.List[object]
    foreach ($item in $g.Group) {
        if ($len + $item.File.Length + 3 -gt 28000 -and $batch.Count -gt 0) {
            $batches.Add($batch.ToArray()); $batch.Clear(); $len = 0
        }
        $batch.Add($item.File); $len += $item.File.Length + 3
    }
    if ($batch.Count -gt 0) { $batches.Add($batch.ToArray()) }

    $groupOk = $true
    foreach ($b in $batches) {
        $out = & $TexConv @tcArgs @b 2>&1
        if ($LASTEXITCODE -ne 0) {
            $groupOk = $false
            Write-Warning ("texconv failed in {0}: {1}" -f $first.Folder, ($out | Select-Object -Last 3 | Out-String).Trim())
        }
    }

    if ($groupOk) {
        # Match outputs back to sources by stem; texconv may not preserve extension case.
        $produced = @{}
        foreach ($o in (Get-ChildItem -LiteralPath $outDir -File -ErrorAction SilentlyContinue)) {
            $produced[[System.IO.Path]::GetFileNameWithoutExtension($o.Name).ToLowerInvariant()] = $o.FullName
        }
        foreach ($item in $g.Group) {
            $key = $item.Stem.ToLowerInvariant()
            if (-not $produced.ContainsKey($key)) {
                Write-Warning "no texconv output for $($item.File)"
                $failed++
                continue
            }
            if ($BackupRoot) {
                $rel = $item.File -replace '^[A-Za-z]:[\\/]', ''
                $dst = Join-Path $BackupRoot $rel
                New-Item -ItemType Directory -Path (Split-Path -Parent $dst) -Force | Out-Null
                Copy-Item -LiteralPath $item.File -Destination $dst -Force
                $manifest.Add([pscustomobject]@{
                    Package = $item.Package; OriginalPath = $item.File; BackupPath = $dst
                })
            }
            # Write back under the exact original file name, leaving the .DDS.json sidecar alone.
            Copy-Item -LiteralPath $produced[$key] -Destination $item.File -Force
            $done++
        }
    } else {
        $failed += $g.Count
    }

    Remove-Item -LiteralPath $outDir -Recurse -Force -ErrorAction SilentlyContinue
}
Write-Progress -Activity 'Re-encoding textures' -Completed
Remove-Item -LiteralPath $tempRoot -Recurse -Force -ErrorAction SilentlyContinue

if ($BackupRoot -and $manifest.Count -gt 0) {
    $manifest | Export-Csv -LiteralPath (Join-Path $BackupRoot 'restore-manifest.csv') -NoTypeInformation -Encoding UTF8
}

Write-Host ''
Write-Host ("Re-encoded {0:N0} textures, {1:N0} failed." -f $done, $failed) -ForegroundColor $(if ($failed) { 'Yellow' } else { 'Green' })

# ------------------------------------------------------------------ layout ---

Write-Host ''
if ($LayoutGenerator) {
    $regen = 0; $skipped = 0
    foreach ($pkg in $packages) {
        $layout = Join-Path $pkg.FullName 'layout.json'
        if (-not (Test-Path -LiteralPath $layout)) {
            Write-Warning "no layout.json in $($pkg.Name) - skipping"
            $skipped++
            continue
        }
        Write-Host "Regenerating layout.json for $($pkg.Name)"
        & $LayoutGenerator $layout | Out-Null
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "layout regeneration reported an error for $($pkg.Name)"
            $skipped++
        } else {
            $regen++
        }
    }
    Write-Host ("layout.json regenerated for {0} of {1} packages." -f $regen, $packages.Count) `
               -ForegroundColor $(if ($skipped) { 'Yellow' } else { 'Green' })
    if ($skipped) {
        Write-Host "$skipped package(s) were not updated - check the warnings above before launching MSFS." -ForegroundColor Yellow
    }
} else {
    Write-Host 'ACTION REQUIRED' -ForegroundColor Red
    Write-Host 'Regenerate layout.json in every modified package before launching MSFS.' -ForegroundColor Red
    Write-Host 'Drag each layout.json onto MSFSLayoutGenerator.exe, or re-run with -LayoutGenerator.' -ForegroundColor Red
}
Write-Host ''
