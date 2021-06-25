# Find My documentation build configuration file

import os
from pathlib import Path
import sys


# Paths ------------------------------------------------------------------------

NRF_BASE = os.environ.get("NRF_BASE")
if not NRF_BASE:
    raise FileNotFoundError("NRF_BASE not defined")
NRF_BASE = Path(NRF_BASE)

FIND_MY_BASE = os.environ.get("FIND_MY_BASE")
if not FIND_MY_BASE:
    raise FileNotFoundError("FIND_MY_BASE not defined")
FIND_MY_BASE = Path(FIND_MY_BASE)

sys.path.insert(0, str(NRF_BASE / "doc" / "_utils"))
import utils

# General configuration --------------------------------------------------------

project = "Find My support for nRF Connect SDK"
copyright = "2021, Nordic Semiconductor"

sys.path.insert(0, str(NRF_BASE / "doc" / "_extensions"))

extensions = ["sphinx.ext.intersphinx", "recommonmark", "external_content"]
source_suffix = [".rst"]
exclude_patterns = ["**/release-notes-latest.rst"]

# Options for HTML output ------------------------------------------------------

html_theme = "sphinx_ncs_theme"
html_static_path = [str(NRF_BASE / "doc" / "_static")]
html_last_updated_fmt = "%b %d, %Y"
html_show_sourcelink = True
html_show_sphinx = False

# Options for external_content -------------------------------------------------

external_content_contents = [
    (FIND_MY_BASE, "./**/*.rst")
]

def setup(app):
    app.add_css_file("css/common.css")
    app.add_css_file("css/find-my.css")
