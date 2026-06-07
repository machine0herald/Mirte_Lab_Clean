# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Lab Cleanup Robot using MIRTE Master Platform'
copyright = '2026, Matthew de Lannoy'
author = 'Matthew de Lannoy'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    # 'sphinx.ext.intersphinx',
]

html_theme = 'sphinx_rtd_theme'

# Link to each package's built docs for cross-referencing
# intersphinx_mapping = {
#     'mirte_lc_gazebo':      ('https://matt-rbt.github.io/Mirte_Lab_Clean/mirte_lc_gazebo', None),
#     'mirte_lc_labclean':    ('https://matt-rbt.github.io/Mirte_Lab_Clean/mirte_lc_labclean', None),
#     'mirte_lc_moveit_cpp':  ('https://matt-rbt.github.io/Mirte_Lab_Clean/mirte_lc_moveit_cpp', None),
#     'mirte_lc_msgs':        ('https://matt-rbt.github.io/Mirte_Lab_Clean/mirte_lc_msgs', None),
#     'mirte_lc_nav2':        ('https://matt-rbt.github.io/Mirte_Lab_Clean/mirte_lc_nav2', None),
#     'mirte_lc_vision':      ('https://matt-rbt.github.io/Mirte_Lab_Clean/mirte_lc_vision', None),
# }

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output
html_static_path = ['_static']
html_baseurl = 'https://matt-rbt.github.io/Mirte_Lab_Clean/'
