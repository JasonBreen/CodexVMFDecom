# CodexVMFDecom

A basic Source `.bsp` decompiler that exports a raw `.vmf` with brush solids and texture names preserved where possible.

## Features

- Reads Source BSP (`VBSP`) headers and core lumps.
- Reconstructs world brushes from planes/brush sides.
- Maps brush side materials from `texinfo` + `texdata` + texture string tables.
- Exports point entities from the BSP entity lump.

## Usage

```bash
python3 bsp_to_vmf.py path/to/map.bsp path/to/output.vmf
```

If no output path is provided, the script writes next to the BSP file with a `.vmf` extension.

## Notes / limits

- Intended as a **basic** geometry/material recovery pass.
- Does not currently rebuild displacements, overlays, areaportals, vis clusters, or brush entity solids.
- Texture axes are carried from BSP texinfo; final alignment may still need editor cleanup.
