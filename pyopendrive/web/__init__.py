"""Local API server for the MapLibre OpenDRIVE editor.

The browser understands MapLibre layers and GeoJSON.  pyopendrive understands
OpenDRIVE roads, lanes, signals, and local ``x/y`` map coordinates.  This
module sits between those two worlds:

1. Read an ``.xodr`` file with :func:`readXodr`.
2. Convert road, lane, and signal geometry to GeoJSON in longitude/latitude.
3. Accept edited GeoJSON from the browser.
4. Write the supported edits back into the original OpenDRIVE XML.

The save path is intentionally conservative.  It updates the XML elements that
the web page can edit today, while leaving unknown OpenDRIVE content untouched.
"""

from __future__ import annotations
from ._editor import xodr_web_viewer, run_server


__all__ = ["xodr_web_viewer", "run_server"]
