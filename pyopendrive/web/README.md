# OpenDRIVE Web Viewer

This folder contains the browser viewer for OpenDRIVE `.xodr` files.

The viewer is a static Emscripten/WebAssembly application. It loads the compiled
libOpenDRIVE viewer from `viewer.js`, `viewer.wasm`, and `viewer.data`, renders
with WebGL, and processes selected `.xodr` files locally in the browser.

## Files

- `index.html` - viewer page and file-upload UI.
- `viewer.js` - Emscripten JavaScript loader.
- `viewer.wasm` - compiled WebAssembly module.
- `viewer.data` - preloaded viewer data package. It includes the default
  `/data.xodr` map used at startup.
- `viewer.local-assets.js` - embedded `viewer.wasm` and `viewer.data` fallback
  used only when `index.html` is opened directly as a local `file://` page.
- `fonts/` - local Bootstrap and Font Awesome assets used by the page.
- `favicon.ico` - browser icon.

All files above must stay together. If `viewer.wasm` or `viewer.data` is missing,
the page will load but the viewer runtime will fail.

## Run Locally

The viewer supports both normal HTTP serving and direct local opening.

### Option 1: Local HTTP Server

This is the recommended development path because the browser loads
`viewer.wasm` and `viewer.data` as separate files.

From the repository root:

```powershell
python -m http.server 8765 --bind 127.0.0.1
```

Then open:

```text
http://127.0.0.1:8765/web/index.html
```

Alternatively, serve from inside the `web` folder:

```powershell
cd web
python -m http.server 8765 --bind 127.0.0.1
```

Then open:

```text
http://127.0.0.1:8765/index.html
```

### Option 2: Open `index.html` Directly

You can also double-click `web/index.html` or open it from the browser with a
`file://` URL. In this mode, `index.html` loads `viewer.local-assets.js`, which
contains embedded copies of `viewer.wasm` and `viewer.data`. This avoids browser
restrictions on fetching WebAssembly/data files from local disk.

Keep these files in the same folder for local mode:

- `index.html`
- `viewer.js`
- `viewer.local-assets.js`
- `fonts/`
- `favicon.ico`

## Use The Viewer

1. Open the page in a browser with WebGL and WebAssembly support.
2. Wait for the default map from `viewer.data` to load.
3. Click `Open .xodr` to choose another OpenDRIVE file.
4. Navigate the scene:
   - Pan: left mouse drag.
   - Zoom: mouse wheel.
   - Orbit: right mouse drag, or left mouse drag with `Ctrl`/`Shift`.

The selected file is read through the browser `FileReader` API and placed into
the in-memory Emscripten filesystem as `data.xodr`. It is not uploaded.

## Verified Capability

Current verified behavior:

- Loads `index.html`, `viewer.js`, `viewer.wasm`, `viewer.data`, and local CSS
  assets from a local HTTP server.
- Opens from local disk through `file://` using `viewer.local-assets.js`.
- Starts the WebAssembly runtime through `OdrViewer()`.
- Loads the packaged default `/data.xodr` map at startup.
- Supports opening local `.xodr` or `.xml` files from the file picker.
- Refits selected file header bounds (`west`, `east`, `south`, `north`) from
  the road plan-view geometry before loading, so the viewer base/grid mesh uses
  the opened map boundary instead of stale source extents.
- Restarts the viewer after file selection and injects the selected `.xodr` as
  the initial runtime map. The handoff uses IndexedDB so larger local maps can
  survive the reload. This is necessary because the compiled grid mesh is built
  during the first `load_map()` call and is not rebuilt by later map swaps.
- Renders through WebGL on the page canvas.
- Supports mouse pan, zoom, and orbit controls.

This folder does not contain a build pipeline. The WebAssembly artifacts are
prebuilt outputs; regenerating them requires the original Emscripten/libOpenDRIVE
web build setup.

## Troubleshooting

If the page stays on `Loading...`, check the browser developer console and make
sure these requests return HTTP 200:

- `viewer.js`
- `viewer.wasm`
- `viewer.data`
- `viewer.local-assets.js` when opened through `file://`
- `fonts/font-awesome.min.css`
- `fonts/bootstrap.min.css`

If WebGL fails, try a current Chrome, Edge, or Firefox browser and verify that
hardware acceleration/WebGL is enabled.

If a new map does not load, validate that the file is a readable OpenDRIVE XML
file and has an `.xodr` or `.xml` extension.
