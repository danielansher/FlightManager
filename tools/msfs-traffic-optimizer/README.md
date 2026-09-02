# MSFS 2020 AI Traffic Texture Optimizer

Tooling to inventory and shrink the textures in AIG / FSLTL / FSTraffic AI-traffic
packages, aimed at reclaiming VRAM when running BeyondATC traffic injection.

> These are Windows PowerShell scripts, unrelated to the Java FlightManager app in
> this repo's `src/`. They live here only so they are versioned and easy to fetch.

---

## Read this before you re-encode anything

Shrinking textures fixes exactly one problem: **VRAM exhaustion**. It does nothing
for the other, more common cause of AI-traffic frame loss.

MSFS 2020 is bottlenecked on a single main thread. Every injected aircraft costs
draw calls and CPU time on that thread *regardless of how large its textures are*.
If your frame loss is CPU-bound, downscaling every texture you own to 512px will
buy you approximately zero FPS, after several hours of encoding.

So diagnose first:

1. Load into a busy hub with BeyondATC running its normal traffic level.
2. Open MSFS Developer Mode → `Options` → `Display FPS`, and watch the counter.
   - Bar reads **MainThread** limited, VRAM comfortable → CPU-bound. Textures are
     not your problem; cut aircraft *count* instead (steps 1–3 below).
   - VRAM at or near your card's limit, with stutters and sudden drops that get
     worse the longer you sit at the gate → memory-bound. Texture reduction is
     the right fix and will help a lot.
3. Cross-check with MSI Afterburner / RTSS for dedicated VRAM usage over time.

### Do these first — they are free and cost no visual quality

1. **Run only one model library.** You have AIG, FSLTL *and* FSTraffic installed.
   BeyondATC matches against one model set at a time (check its Traffic / model
   matching setting). The other two are pure cost: package scan time at startup,
   disk, and VRAM whenever something matches into them. Disable the two you are
   not matching against — move them out of `Community`, or use MSFS Addons Linker
   so you can flip them back. **This is the single biggest win available to you,
   and it is reversible in seconds.**
2. **Turn down BeyondATC's traffic density / maximum aircraft.** Going from ~80
   to ~40 aircraft at a hub roughly halves the traffic cost on the main thread.
   Texture size cannot touch this number.
3. **Set the sim's own traffic to Off.** `Options → General → Traffic → Aircraft
   Traffic Type = Off`. Injected traffic comes in over SimConnect; leaving the
   built-in generator on stacks a second set of aircraft on top of BATC's.
   Also drop Ground Aircraft Density and Airport Life if you are CPU-bound.
4. **Prune liveries you never see.** AIG OCI installs per-airline. A hundred
   airlines you never encounter is a hundred livery packages MSFS enumerates at
   startup. Keep the regions you actually fly.
5. **Only then** re-encode textures.

---

## Requirements

| Tool | Why | Where |
|---|---|---|
| `texconv.exe` | DDS resize + re-encode | [DirectXTex releases](https://github.com/microsoft/DirectXTex/releases) |
| `MSFSLayoutGenerator.exe` | Rebuild `layout.json` | [MSFSLayoutGenerator releases](https://github.com/HughesMDflyer4/MSFSLayoutGenerator/releases) |

PowerShell 5.1 (ships with Windows) is enough.

### The layout.json rule

Every MSFS package carries a `layout.json` listing each file's size and date. Change
a file without rebuilding it and the sim will ignore your changes or crash on load.
**Regenerating `layout.json` after modifying textures is not optional.**
`Optimize-MsfsTrafficTextures.ps1 -LayoutGenerator ...` does it for you.

---

## Step 1 — Scan (read-only, changes nothing)

```powershell
.\Scan-MsfsTraffic.ps1
```

It auto-detects your Community folder from `UserCfg.opt` (both Steam and MS Store
layouts), finds every package whose name contains `aig`, `fsltl` or `fstraffic`,
reads the DDS header of every texture, and reports:

- size and texture count per package
- a resolution histogram — **this is the number that decides everything**
- the heaviest texture folders
- projected savings at your chosen caps
- a warning listing the overlapping model libraries you have active

Override the detection if needed:

```powershell
.\Scan-MsfsTraffic.ps1 -CommunityPath 'D:\MSFS\Community' -TargetAlbedo 1024 -TargetOther 512
```

**How to read the histogram.** If most textures are already 1024 or below, there is
little to win and you should stop here — AIG's models are largely FSX-era ports and
are often already modest. If you see a large mass at 2048 and 4096, capping albedo
at 1024 cuts those files to a quarter and a sixteenth of their size respectively,
and that is worth doing.

A per-file CSV lands next to the script for sorting in Excel.

## Step 2 — Dry run

```powershell
.\Optimize-MsfsTrafficTextures.ps1 `
    -PackagePath 'D:\MSFS\Community\aig-*' `
    -TexConv 'C:\tools\texconv.exe'
```

Prints the plan and the savings. Writes nothing without `-Apply`.

## Step 3 — Apply

```powershell
.\Optimize-MsfsTrafficTextures.ps1 `
    -PackagePath 'D:\MSFS\Community\aig-*','D:\MSFS\Community\fsltl-traffic-base' `
    -TexConv 'C:\tools\texconv.exe' `
    -LayoutGenerator 'C:\tools\MSFSLayoutGenerator.exe' `
    -BackupRoot 'E:\msfs-texture-backup' `
    -MaxAlbedo 1024 -MaxOther 512 `
    -UseGpu -Apply
```

## Step 4 — Roll back if you dislike it

```powershell
.\Optimize-MsfsTrafficTextures.ps1 -BackupRoot 'E:\msfs-texture-backup' `
    -LayoutGenerator 'C:\tools\MSFSLayoutGenerator.exe' -Restore -Apply
```

---

## What the optimizer actually does

- Reads each DDS header for dimensions and DXGI/FourCC format.
- Picks a **power-of-two** downscale so the long edge lands at or under the cap,
  preserving aspect ratio and never going below one 4×4 compression block.
- Re-encodes with texconv **into the same format** the texture already used — a
  `BC7_UNORM_SRGB` albedo stays `BC7_UNORM_SRGB`, a `BC5_UNORM` normal map stays
  `BC5_UNORM` — with a full mip chain regenerated (`-m 0`).
- Applies gamma-correct filtering (`-srgb`) for sRGB formats.
- Leaves every `.DDS.json` sidecar untouched.
- Skips cube maps, volume textures and anything whose format it cannot identify,
  rather than guessing.
- Backs each original up and writes a `restore-manifest.csv` mapping backups to
  their original paths.
- Regenerates `layout.json` per package at the end.

### Why two separate caps

`-MaxAlbedo` (default 1024) governs colour. `-MaxOther` (default 512) governs
normal, composite, mask, lightmap and similar data textures, matched by MSFS's
`_NORM` / `_COMP` / `_MASK` naming and the AIG legacy `_bump` / `_lm` conventions.

You genuinely cannot see normal-map detail on an aircraft 400 m away on a taxiway,
so those are the cheapest bytes in the whole package. If you want to be aggressive,
`-MaxAlbedo 512 -MaxOther 256` still looks fine in the air and only gets soft on
the aircraft parked directly beside you at the gate.

### Expected result

Block-compressed textures occupy roughly their on-disk size in VRAM, so the scan's
"saved" figure is a fair estimate of VRAM reclaimed. Capping a 2048-heavy library
at 1024 typically removes 60–75% of the texture bytes.

If you were VRAM-limited, that removes the stutter. If you were main-thread limited,
it will not, which is why the diagnosis at the top matters more than this script.

---

## Caveats

- **Updates overwrite your work.** AIG Manager / OCI and FSLTL updates replace
  package contents. Re-run after any content update.
- **AIG OCI may flag modified packages** during a verify/repair pass and re-download.
- **FSTraffic is a paid Just Flight product** — fine to modify for personal use, but
  its installer will restore originals on repair.
- Back up. `-BackupRoot` costs disk and is worth it.
- BC7 CPU encoding is slow — a large library can take hours. `-UseGpu` is roughly
  an order of magnitude faster and visually equivalent at these sizes.
