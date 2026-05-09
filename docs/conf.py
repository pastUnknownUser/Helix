# -- Project information -----------------------------------------------------
project = 'Helix Robotics'
copyright = '2026, Team Helix'
author = 'Team Helix'
release = '1.0.1' # Incremented for fresh build

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',    # For nice Google/NumPy style docstrings
    'sphinx.ext.githubpages',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'

# Path for static assets (Critical for overriding theme assets)
html_static_path = ['_static']

# Inject custom stylesheet (This file must exist)
html_css_files = [
    'css/custom.css',
]

# Standard Read the Docs Theme options, but modified for our glass base
html_theme_options = {
    'style_nav_header_background': 'transparent', # Glass looks better with transparent header
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

# The master toctree document
master_doc = 'index'