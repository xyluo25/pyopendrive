'''
##############################################################
# Created Date: Thursday, April 30th 2026
# Contact Info: luoxiangyong01@gmail.com
# Author/Copyright: Mr. Xiangyong Luo
##############################################################
'''

from __future__ import annotations

from pathlib import Path
import webbrowser


WEB_DIR = Path(__file__).resolve().parent
INDEX_HTML = WEB_DIR / "index.html"

__all__ = ["web_viewer"]


def web_viewer(index_html: str | Path = INDEX_HTML) -> Path:
    """Open the bundled viewer in the default browser.

    Args:
        index_html: Path to the HTML file to open. Defaults to the bundled viewer page.

    Returns:
        The resolved path that was opened.
    """
    index_path = Path(index_html).resolve()
    webbrowser.open(index_path.as_uri())
    return index_path
