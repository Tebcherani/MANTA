import re


project = "MANTA"
extensions = ["myst_parser", "sphinxcontrib.plantuml"]
source_suffix = {".md": "markdown", ".rst": "restructuredtext"}
root_doc = "index"
exclude_patterns = []
myst_heading_anchors = 6

html_theme = "furo"
html_title = "MANTA Documentation"
html_static_path = ["_static"]
html_css_files = ["manta.css"]
html_show_copyright = False
html_show_sourcelink = False
html_theme_options = {
    "light_css_variables": {
        "color-brand-primary": "#002D72",
        "color-brand-content": "#002D72",
        "color-background-primary": "#FFFFFF",
        "color-background-secondary": "#F5F7FA",
        "color-foreground-primary": "#24292F",
    },
    "dark_css_variables": {
        "color-brand-primary": "#68ACE5",
        "color-brand-content": "#68ACE5",
    },
}
pygments_style = "friendly"
pygments_dark_style = "monokai"

plantuml = (
    "java -Djava.awt.headless=true -Duser.home=/tmp -DPLANTUML_SECURITY_PROFILE=SANDBOX "
    "-jar /opt/plantuml.jar -charset UTF-8"
)
plantuml_output_format = "svg_img"
plantuml_cache_path = "/output/cache/plantuml"
plantuml_syntax_error_image = False


def link_diagrams(app, pagename, templatename, context, doctree):
    context["body"] = re.sub(
        r'(<p class="plantuml">\s*)(<img src="([^"]+)"[^>]*>)',
        r'\1<a href="\3">\2</a>',
        context.get("body", ""),
    )


def setup(app):
    app.connect("html-page-context", link_diagrams)
