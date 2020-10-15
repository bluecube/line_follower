The robot has a PCB base with 3D printed parts bolted to it.

All 3D printed parts should be printable without supports, although
some parts are a little more difficult:

- Deck is printed standing on the (only) flat face. It is tall and overhanging, good cooling is necesary. Orient the longer side of the base along the movement of the Y axis.
- Motor bracket has long bridges supported only by thin columns.

The mechanical design was done in a little roundabout way:

1. 2D sketches of the PCB were done in F360 (`board-base-sketches.f3d`)
2. The base sketches were exported to DXF files in the `pcb-exports/` directory.
3. PCB was designed based on these exported DXF files (`../kicad/`)
4. The PCB design was then exported as STEP, imported into F360 and used as a base for most of the 3D modelling (`lf.f3z`).
5. Various outputs were then exported from F360 for fabrication (`output-exports/`)
