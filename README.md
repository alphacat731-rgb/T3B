# T3B

A tiny CPU-only Linux terminal 3D model viewer for Raspberry Pi hardware, including the Pi 3/3B.

## What it does

T3B renders a 3D model directly inside a text terminal. The default Braille wireframe mode uses 2x4 terminal subpixels, so edges are much finer than a normal one-character ASCII renderer. Press 1 to switch to classic dot ASCII mode.

### Built-in formats

- OBJ
- GLB / glTF 2.0
- STL (ASCII + binary)
- PLY (ASCII)

### Assimp formats

When the assimp command is installed, T3B can use it as a conversion bridge for formats outside the native loaders, including FBX and many other common 3D formats. Assimp supports 40+ import formats.

## Install on Debian Trixie

    sudo apt update
    sudo apt install python3 assimp-utils

Clone and run:

    git clone https://github.com/alphacat731-rgb/T3B.git
    cd T3B
    python3 t3b.py path/to/model.glb

No desktop environment, X11, Wayland, SDL or GPU is required for the viewer itself.

For maximum compatibility with terminals that cannot display Braille correctly:

    python3 t3b.py --ascii path/to/model.obj

Run the built-in cube as a quick test:

    python3 t3b.py

## Controls

| Key | Action |
|---|---|
| Arrow keys | Rotate |
| W / S | Zoom in / out |
| A / D | Pan left / right |
| Z / C | Roll |
| Space | Toggle auto-rotate |
| R | Reset view |
| 1 | Braille / ASCII |
| 2 | Change terminal colour |
| H / ? | Help |
| Q / Esc | Quit |

## Performance notes

T3B deliberately avoids a heavyweight GUI stack. The renderer is CPU-only and caps the number of visible wireframe edges with --edges so detailed assets remain usable on small ARM boards.

Examples:

    python3 t3b.py model.fbx --edges 8000 --fps 20
    python3 t3b.py model.glb --edges 18000 --fps 30

For very dense models, lower --edges first. The importer can still load the full mesh while the terminal renderer displays a sampled edge set.

## Roadmap

- Better mesh reduction for ultra-dense assets
- Optional solid/ASCII shading
- Basic material/texture metadata
- Animation playback
- Terminal file picker
- Optional GPU renderer for Pi models with a graphics stack
