# -- Project information -----------------------------------------------------
project = 'Helix Robotics'
copyright = '2026, Team Helix'
author = 'Team Helix'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.githubpages',
]

templates_path = ['_templates']
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'

# Path for static assets (CSS, JS, Images)
html_static_path = ['_static']

# Inject custom stylesheet
html_css_files = [
    'css/custom.css',
]

# Theme options
html_theme_options = {
    'style_nav_header_background': 'transparent', # Make sidebar header transparent
    'collapse_navigation': False,
    'sticky_navigation': True,
}