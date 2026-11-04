"""Prepare selected documentation inputs without modifying their sources."""

import os
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote, urlsplit


CONTENT_SUFFIXES = {".md", ".rst", ".puml"}
ASSET_SUFFIXES = {
    ".png", ".jpg", ".jpeg", ".gif", ".svg", ".webp", ".pdf", ".iuml",
}
DOWNLOAD_SUFFIXES = ASSET_SUFFIXES | {
    ".yaml", ".yml", ".json", ".csv", ".toml", ".txt", ".mi", ".vd", ".fd",
}
MARKDOWN_LINK = re.compile(r"!?\[[^\]]*\]\(\s*(?:<([^>]+)>|([^\s)]+))")


def eligible(relative):
    parts = relative.parts
    return (
        not any(part.startswith(".") for part in parts)
        and parts[:2] != ("missions", "vehicle_only")
        and parts[:2] != ("docs", "sphinx")
    )


def write_changed(path, data):
    if path.is_file() and path.read_bytes() == data:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_bytes(data)
    os.replace(temporary, path)
    return True


def toc(caption, entries):
    if not entries:
        return ""
    return (
        "```{toctree}\n:caption: " + caption + "\n:maxdepth: 1\n\n"
        + "\n".join(entries) + "\n```"
    )


def prepare(root):
    root = Path(root).resolve()
    tooling = root / "docs/sphinx"
    build = root / ".build/docs"
    source = build / "source"
    config = build / "config"
    html = build / "html"
    for directory in (source, config, html, build / "cache"):
        directory.mkdir(parents=True, exist_ok=True)

    files = {}
    readme = subprocess.run(
        ["git", "show", "HEAD:README.md"], cwd=root, check=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    ).stdout
    files[Path("README.md")] = readme
    folders = [root / "docs"]
    for parent in (root / "components", root / "missions"):
        folders.extend(parent.glob("*/docs"))

    candidates = [path for folder in folders for path in folder.rglob("*")]
    candidates.extend((root / "missions").glob("*/README.md"))
    for path in candidates:
        relative = path.relative_to(root)
        if not eligible(relative) or path.suffix.lower() not in CONTENT_SUFFIXES | ASSET_SUFFIXES:
            continue
        if path.is_symlink() or root not in path.resolve().parents:
            raise ValueError(f"Documentation input must stay inside the repository: {relative}")
        if path.is_file():
            files[relative] = path.read_bytes()

    # Collect only explicitly referenced local downloads, never arbitrary repo files.
    for relative, data in list(files.items()):
        if relative.suffix.lower() != ".md":
            continue
        for match in MARKDOWN_LINK.finditer(data.decode("utf-8")):
            target = urlsplit(match.group(1) or match.group(2))
            if target.scheme or target.netloc or not target.path:
                continue
            linked = (root / relative.parent / unquote(target.path)).resolve()
            if not linked.is_relative_to(root):
                raise ValueError(f"Documentation link leaves the repository: {relative}")
            linked_relative = linked.relative_to(root)
            if not eligible(linked_relative):
                raise ValueError(f"Documentation links to an excluded path: {linked_relative}")
            if linked.suffix.lower() in DOWNLOAD_SUFFIXES:
                if not linked.is_file():
                    raise ValueError(f"Missing documentation asset: {linked_relative}")
                files[linked_relative] = linked.read_bytes()

    sections = {"overview": ["README"], "development": [], "components": [], "missions": []}
    groups = {}
    for relative, data in sorted(list(files.items())):
        if relative == Path("README.md"):
            continue
        if relative.suffix.lower() not in CONTENT_SUFFIXES:
            continue
        section = {
            "docs": "development", "components": "components", "missions": "missions",
        }[relative.parts[0]]
        if relative.suffix.lower() == ".puml":
            text = data.decode("utf-8")
            match = re.search(r"^title\s+(.+)$", text, re.MULTILINE)
            title = match.group(1).strip() if match else relative.stem
            wrapper = Path("diagrams") / relative.with_suffix(".rst")
            files[wrapper] = (
                title + "\n" + "=" * len(title) + "\n\n"
                + ".. uml:: /" + relative.as_posix() + "\n"
                + "   :alt: " + title + "\n"
            ).encode("utf-8")
            document = wrapper.with_suffix("").as_posix()
        else:
            document = relative.with_suffix("").as_posix()
        if section == "development":
            sections[section].append(document)
        else:
            groups.setdefault((section, relative.parts[1]), []).append(document)

    for (section, name), entries in sorted(groups.items()):
        directory = Path("directories") / section / (name + ".md")
        files[directory] = (
            "# " + name + "\n\n" + toc(name, ["/" + entry for entry in entries]) + "\n"
        ).encode("utf-8")
        sections[section].append(directory.with_suffix("").as_posix())

    index = (tooling / "index.md").read_text(encoding="utf-8")
    for section, entries in sections.items():
        index = index.replace("{{" + section + "}}", toc(section.title(), entries))
    files[Path("index.md")] = index.encode("utf-8")

    changed = False
    for path in source.rglob("*"):
        if path.is_file() and path.relative_to(source) not in files:
            if path.suffix.lower() in {".md", ".rst"}:
                (html / path.relative_to(source).with_suffix(".html")).unlink(missing_ok=True)
            path.unlink()
            changed = True
    for relative, data in files.items():
        changed = write_changed(source / relative, data) or changed

    for relative in (Path("conf.py"), Path("_static/manta.css")):
        changed = write_changed(config / relative, (tooling / relative).read_bytes()) or changed
    return changed
