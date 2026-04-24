# CodexVMFDecom

A basic Source `.bsp` decompiler that exports a raw `.vmf` with brush solids and texture names preserved where possible.

## Features

- Reads Source BSP (`VBSP`) headers and core lumps.
- Handles Source SDK 2013-compatible BSP brushside layout (`dbrushside_t` 8-byte format).
- Supports Valve-style LZMA-compressed lumps (commonly seen in newer Source branches and tools).
- Reconstructs world brushes from planes/brush sides.
- Maps brush side materials from `texinfo` + `texdata` + texture string tables.
- Normalizes cubemap-patched `maps/<map>/...` material names back to base materials to reduce missing textures in Hammer.
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

## References

- Valve Source SDK 2013 `bspfile.h` (lump and struct definitions).
- Valve Developer Community BSP (Source) format documentation.
