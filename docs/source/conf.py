# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os

project = 'Lab Cleanup Robot using MIRTE Master Platform'
copyright = '2026, Matthew de Lannoy'
author = 'Matthew de Lannoy'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

html_theme = 'sphinx_rtd_theme'

# Path from docs/source/ to the docs_output directory
_docs = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../documentation/docs_output'))

extensions = ['sphinx.ext.intersphinx']

intersphinx_mapping = {
    'mirte_lc_gazebo':     ('https://matt-rbt.github.io/Mirte_Lab_Clean/mirte_lc_gazebo',    f'{_docs}/mirte_lc_gazebo/objects.inv'),
    'mirte_lc_labclean':   ('https://matt-rbt.github.io/Mirte_Lab_Clean/mirte_lc_labclean',  f'{_docs}/mirte_lc_labclean/objects.inv'),
    'mirte_lc_moveit_cpp': ('https://matt-rbt.github.io/Mirte_Lab_Clean/mirte_lc_moveit_cpp',f'{_docs}/mirte_lc_moveit_cpp/objects.inv'),
    'mirte_lc_msgs':       ('https://matt-rbt.github.io/Mirte_Lab_Clean/mirte_lc_msgs',      f'{_docs}/mirte_lc_msgs/objects.inv'),
    'mirte_lc_nav2':       ('https://matt-rbt.github.io/Mirte_Lab_Clean/mirte_lc_nav2',      f'{_docs}/mirte_lc_nav2/objects.inv'),
    'mirte_lc_vision':     ('https://matt-rbt.github.io/Mirte_Lab_Clean/mirte_lc_vision',    f'{_docs}/mirte_lc_vision/objects.inv'),
}
templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
html_static_path = ['_static']
