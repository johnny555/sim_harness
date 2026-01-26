# Configuration file for the Sphinx documentation builder.
#
# For a full list of configuration options see:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys

# -- Path setup --------------------------------------------------------------

# Add the package's source directory to sys.path for autodoc
sys.path.insert(0, os.path.abspath('../sim_harness'))

# -- Project information -----------------------------------------------------

project = 'sim_harness'
copyright = '2024, John Vial'
author = 'John Vial'
release = '0.1.0'
version = '0.1.0'

# -- General configuration ---------------------------------------------------

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.napoleon',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
]

# Napoleon settings for Google/NumPy style docstrings
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = True

# Autodoc settings
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
    'exclude-members': '__weakref__'
}
autodoc_mock_imports = [
    'rclpy',
    'rclpy.node',
    'rclpy.action',
    'std_msgs',
    'geometry_msgs',
    'sensor_msgs',
    'nav_msgs',
    'nav2_msgs',
    'visualization_msgs',
    'tf2_ros',
    'tf2_geometry_msgs',
    'launch',
    'launch_ros',
    'action_msgs',
    'pytest',
]

# Intersphinx configuration
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
}

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# The root document
root_doc = 'index'

# -- Options for HTML output -------------------------------------------------

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

html_theme_options = {
    'navigation_depth': 4,
    'titles_only': False,
    'collapse_navigation': False,
}

# Hide copyright notice in footer
html_show_copyright = False

# -- Options for todo extension ----------------------------------------------

todo_include_todos = True
