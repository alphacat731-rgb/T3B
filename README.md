# T3B

A tiny CPU-only Linux terminal 3D model viewer for Raspberry Pi hardware, including the Pi 3/3B.

T3B is designed around a very simple idea: **throw a real 3D mesh into a normal text terminal and make it look good.**

## Current renderer

The default renderer is a **fine Braille wireframe**. A Braille terminal cell provides a 2x4 dot grid, giving much thinner-looking edges than traditional one-character ASCII wireframes.

The viewer also includes a classic ASCII-dot mode for terminals where Braille characters are unavailable or render badly.

Imported geometry is automatically recentered and fitted to the viewport, so assets with unusual scene scales should start out filling the screen instead of appearing as a tiny speck.

For dense meshes, T3B scores edges using length, mesh borders and sharp creases, then samples the remaining topology. This keeps more visually useful structure when the renderer has to reduce the edge budget.

T3B also caches the rendered model frame while the camera is idle. A stationary model therefore does not continuously re-rasterize itself just to update the terminal.

### Adaptive quality

The default AUTO quality mode starts at a moderate edge budget and adjusts it toward a target render rate. It reduces geometry when the machine is overloaded and gradually restores detail when there is headroom.

Press K during viewing to cycle through AUTO, HIGH, MED and LOW. HIGH uses the full configured edge budget for maximum wireframe detail; AUTO is intended for small boards such as the Raspberry Pi 3B.

## Formats

### Native loaders

- OBJ
- GLB / glTF 2.0
- STL (ASCII + binary)
- PLY (ASCII)

### Assimp bridge

When the \`assimp\` command is installed, T3B uses it as a conversion bridge for formats outside the native loaders, including **FBX** and many other common 3D formats.

On Debian Trixie:

    sudo apt update
    sudo apt install python3 assimp-utils

## Included FBX test models

The repository currently contains these assets for testing Assimp/FBX loading:

- \`Dupin.fbx\`
- \`Katress.fbx\`
- \`Roblox Pur03.fbx\`
- \`Wenda.fbx\`

Example:

    python3 t3b.py Katress.fbx

Because \`Roblox Pur03.fbx\` contains a space in the filename, quote it normally in the shell:

    python3 t3b.py "Roblox Pur03.fbx"

## Install and run

    git clone https://github.com/alphacat731-rgb/T3B.git
    cd T3B
    sudo apt update
    sudo apt install python3 assimp-utils

Run the built-in cube:

    python3 t3b.py

Load a model:

    python3 t3b.py model.glb
    python3 t3b.py model.obj
    python3 t3b.py model.fbx

For classic ASCII mode:

    python3 t3b.py --ascii model.obj

## Controls

| Key | Action |
|---|---|
| Arrow keys | Rotate |
| W / S | Zoom in / out |
| A / D | Pan left / right |
| Z / C | Roll |
| Space | Toggle auto-rotate |
| R | Reset + refit view |
| 1 | Braille / ASCII |
| 2 | Cycle terminal colour |
| K | Cycle quality: AUTO / HIGH / MED / LOW |
| H / ? | Help |
| Q / Esc | Quit |

## Performance

The renderer is intentionally CPU-only and avoids a desktop GUI stack.

Useful options:

    python3 t3b.py model.fbx --edges 6000 --fps 20
    python3 t3b.py model.glb --edges 12000 --fps 30
    python3 t3b.py model.fbx --quality high

Lower \`--edges\` on very dense models. A larger edge budget gives more mesh detail but costs more CPU time on small ARM boards.

The configured edge budget defaults to 14000. AUTO starts lower on complex meshes and raises/lowers the active budget as performance changes.

## Requirements

- Python 3
- A terminal with curses support
- A terminal that can display Unicode Braille characters for the default mode
- Assimp only for formats outside the native loaders

T3B itself does not require X11, Wayland, SDL or an OpenGL desktop environment.

## Roadmap

- Smarter geometric decimation
- Optional solid/ASCII shading
- Material/colour metadata
- Animation playback
- Terminal file picker
- Optional GPU-backed renderer for systems with a graphics stack
