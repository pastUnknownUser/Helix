# -- Project information -----------------------------------------------------
project = 'Helix'
copyright = '2026, Helix'
author = 'Evan and Logan'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',      # Allows automatic code doc generation
    'sphinx.ext.githubpages',  # Helper for hosting on GitHub pages if desired
]

templates_path = ['_templates']
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'  # The iconic LemLib style theme

# Theme styling tweaks
html_theme_options = {
    'style_nav_header_background': '#1A1A1A',  # Dark header background
    'collapse_navigation': False,
    'sticky_navigation': True,
}