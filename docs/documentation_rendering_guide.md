# Documentation Rendering Guide

MANTA uses Sphinx to build its documentation website, MyST to read Markdown files, and Furo to provide the layout and theme. PlantUML renders the diagrams locally. Docker supplies these tools, so contributors do not need to install them individually.

## Start the local preview

You need Git, Python 3.10 or newer, and a running Docker installation that your user can access. Run the following command from the MANTA repository root:

```bash
./manta docs
```

Open [http://localhost:8000](http://localhost:8000) in your browser. Leave the terminal running while reading or editing documentation. Saving an eligible source file rebuilds the site and refreshes the preview automatically.

Press **Ctrl+C** in that terminal to stop the preview and remove its container.

The first launch downloads and builds the documentation image and requires internet access. Later launches reuse that image. If its Dockerfile or dependency list changes, the next launch rebuilds it. After the image is installed, documentation rendering does not require internet access. External links still require access to their destinations.

The documentation container is separate from the simulator environment. You do not need to build ROS packages or run a mission to view the documentation.

### Use another port

If port 8000 is already in use, select an available port:

```bash
./manta docs --port 8001
```

Then open [http://localhost:8001](http://localhost:8001). A preview already running on port 8000 can also be used directly.

### Command summary

| Command | Result |
| --- | --- |
| `./manta docs` | Start the live preview on port 8000. |
| `./manta docs --port 8001` | Start the live preview on port 8001. |
| `./manta docs build` | Build static HTML and exit. |
| `./manta docs --help` | Show command options. |

The preview is bound to the local computer. Starting it does not publish the documentation to GitHub Pages or the internet.

## Build static HTML

To generate the website without starting a preview server:

```bash
./manta docs build
```

The homepage is written to `.build/docs/html/index.html`. Use the live preview for the normal browser experience, including search. Both commands treat Sphinx warnings as build errors, so check the terminal output before considering a build successful.

Avoid running a static build alongside a preview, or multiple previews from the same checkout: they share the generated sources, HTML, and caches.

## Add or update a documentation page

Keep documentation with the material it describes. The collector discovers the following locations automatically:

| Source location | Navigation section |
| --- | --- |
| `docs/` and its subfolders | Development |
| `components/<component>/docs/` and its subfolders | Components, grouped by component directory |
| `missions/<mission>/docs/` and its subfolders | Missions, grouped by mission directory |
| `missions/<mission>/README.md` | Missions, grouped by mission directory |
| Root `README.md`, read from the current Git commit | Overview |

Use a descriptive filename, such as `documentation_rendering_guide.md`, and begin the file with a single top-level heading:

```markdown
# Documentation Rendering Guide

Opening paragraph.

## First section

Section content.
```

The first heading supplies the page title. Save the file in an eligible location to add it to the site; no manual navigation entry or Sphinx configuration change is needed. Files in those locations can appear before they are committed.

The homepage sections have a fixed order. Within them, generated entries follow source-path order; filenames therefore affect the ordering. Source subfolders are discovered recursively, but the generated navigation groups pages by component or mission, not by every nested folder. Moving or renaming a file updates its generated entry; update any incoming links yourself.

`docs/sphinx/` is reserved for rendering infrastructure and is excluded from ordinary page discovery. Hidden paths and `missions/vehicle_only/` are also excluded. The collector supports existing `.rst` files as well as Markdown.

### Root README behavior

```{note}
The root README is rendered from `HEAD`, the current Git commit. Uncommitted README edits do not appear in the site. This keeps the paused local vehicle-only additions out of the documentation. Other eligible documentation files are read from the working tree.
```

## Write Markdown and MyST

Use ordinary Markdown for paragraphs, lists, emphasis, tables, links, and fenced code blocks. Separate paragraphs and lists with blank lines. Use `##` for sections and `###` for subsections beneath the page's `#` title.

Specify a language after the opening code fence, such as `bash`, `python`, or `yaml`, to enable syntax highlighting.

### Link to another page

Use a relative path to the source Markdown file. For example, from a page directly inside `docs/`:

```markdown
[Code Structure](code_structure.md)
[Component conventions](code_structure.md#components)
```

Sphinx converts these into links to the rendered pages. Keep fragment identifiers aligned with the target heading when headings change. See [Code Structure](code_structure.md) for the existing example.

### Include images and downloads

Keep images inside an eligible documentation directory, usually in an `images/` subfolder, and reference them relative to the page. Supported collected image formats are PNG, JPG, JPEG, GIF, SVG, and WebP; PDF files are also collected.

For example, replace the placeholder image path in this Markdown with an existing local image:

```markdown
![Diagram description][diagram-image]

[diagram-image]: images/example.svg
```

Use meaningful alternative text. Markdown links to local YAML, YML, JSON, CSV, TOML, TXT, MI, VD, and FD files are also collected as downloads when the linked files exist inside the repository and outside excluded paths. Use inline Markdown links for those downloads. Do not link to files outside the repository or to excluded documentation paths.

### Add a note

MyST adds directives to Markdown. A note uses a fenced block with `{note}` after the opening three backticks:

````markdown
```{note}
Text that should stand out from the surrounding explanation.
```
````

Use syntax supported by the current configuration. Additional MyST extensions require an explicit configuration change; installing MyST does not enable every optional feature.

## Add or update a PlantUML diagram

Save a `.puml` file in an eligible documentation directory. For component or mission diagrams, use that component's or mission's `docs/` directory.

```text
@startuml
title Example sequence
Alice -> Bob: Request
Bob --> Alice: Response
@enduml
```

The collector creates a diagram page automatically. A single-line PlantUML `title` becomes its page title; otherwise the filename stem is used. Diagram source files remain in their original locations. Edit those `.puml` files, not generated wrappers or SVGs.

Diagrams render locally to SVG. Click a diagram in the site to open the full-size image. Their white background is retained in both light and dark mode. The renderer uses PlantUML's sandbox security profile; remote includes and other external resources are restricted.

## Rendering files and generated output

| Location | Purpose |
| --- | --- |
| `docs/sphinx/conf.py` | Sphinx extensions, MyST settings, Furo theme, and diagram rendering settings. |
| `docs/sphinx/_static/manta.css` | Shared visual styling and JHU-inspired colors. |
| `docs/sphinx/index.md` | Homepage and navigation placeholders. |
| `docs/sphinx/prepare.py` | Source collection, generated navigation, and diagram wrappers. |
| `docs/sphinx/requirements.txt` | Pinned Python documentation dependencies. |
| `docs/sphinx/Dockerfile` | Documentation image, Java, Graphviz, and PlantUML installation. |
| `components/manta_core/scripts/manta_docs.py` | Command handling, image selection, preview lifecycle, and source watching. |
| `.build/docs/source/` | Collected documentation and generated directory/diagram pages. |
| `.build/docs/config/` | Rendering configuration copied for the container. |
| `.build/docs/html/` | Generated website. |
| `.build/docs/cache/` | Sphinx and diagram caches. |

Edit authoritative sources and rendering configuration. `.build/` is ignored by Git, and generated files can be replaced on the next build.

Changes to collected pages, diagrams, and staged theme settings refresh while the preview runs. Restart the command after changing the launcher, preparation script, Dockerfile, or dependencies. Theme changes belong in the shared configuration and stylesheet so pages keep a consistent appearance.

## Troubleshooting

| Symptom | Action |
| --- | --- |
| Docker is not found or unavailable | Start Docker and confirm your user can access it with `docker info`. |
| The preview port is unavailable | Use the preview already running there, stop its command, or select another port. |
| Browser reports connection refused | Check that the command is still running, the initial build succeeded, and the browser port matches `--port`. |
| A new page is absent | Check its location and title, and inspect the terminal for preparation or Sphinx errors. |
| Root README edits are absent | The renderer uses the committed README from `HEAD`; check the committed version. |
| A link or image fails | Check the relative path, heading fragment, filename case, and whether the target is collected. |
| PlantUML fails | Check the `.puml` syntax and terminal diagnostics. The renderer restricts external resources. |
| The old page remains visible after a failed rebuild | Fix the reported error and save again. An existing HTML page is not proof that the latest build passed. |

Before sharing a documentation change, check the preview, navigation, affected links, and any diagrams. Run `./manta docs build` after stopping the preview to confirm a successful static build.

## References

- [Sphinx documentation](https://www.sphinx-doc.org/en/master/)
- [MyST Markdown syntax](https://myst-parser.readthedocs.io/en/v4.0.1/syntax/typography.html)
- [MyST notes and other admonitions](https://myst-parser.readthedocs.io/en/v4.0.1/syntax/admonitions.html)
- [Furo theme customization](https://pradyunsg.me/furo/customisation/)
- [PlantUML](https://plantuml.com/)
