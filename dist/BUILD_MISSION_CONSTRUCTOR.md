Build instructions — mission-constructor.exe

Purpose
- Repro builds of the single-file `mission-constructor.exe` including data and native DLLs.

Prereqs
- Windows machine (developer account)
- Python venv with the project deps installed (we use `Review\.venv`)
- PyInstaller (tested with 6.20.0)

Quick steps (PowerShell)

1. Activate the venv:

```powershell
& "Review\.venv\Scripts\Activate.ps1"
```

2. Clean leftovers (optional but recommended):

```powershell
Remove-Item -Recurse -Force build\mission-constructor, dist\mission-constructor.exe, mission-constructor.spec -ErrorAction Ignore
```

3. Build (example command used successfully):

```powershell
.":\"Review\\.venv\\Scripts\\python.exe" -m PyInstaller --clean --onefile --windowed --name mission-constructor \
  --add-data "./behaviors;." \
  --add-data "./sensors;sensors" \
  --add-binary "C:/Users/jbellin1/AppData/Local/anaconda3/Library/bin/ffi.dll;." \
  --add-binary "C:/Users/jbellin1/AppData/Local/anaconda3/Library/bin/libcrypto-3-x64.dll;." \
  --add-binary "C:/Users/jbellin1/AppData/Local/anaconda3/Library/bin/libssl-3-x64.dll;." \
  --add-binary "C:/Users/jbellin1/AppData/Local/anaconda3/Library/bin/liblzma.dll;." \
  --add-binary "C:/Users/jbellin1/AppData/Local/anaconda3/Library/bin/libexpat.dll;." \
  --add-binary "C:/Users/jbellin1/AppData/Local/anaconda3/Library/bin/libbz2.dll;." \
  SUtilities/launch_mission_constructor.py
```

Notes on the command
- `--add-data "./behaviors;."` bundles the `behaviors` file into the root of the extracted temp folder.
- `--add-data "./sensors;sensors"` bundles the `sensors` file/dir into a `sensors` folder in the bundle.
- `--add-binary` includes native DLLs PyInstaller may miss (ctypes / OpenSSL / compressors).
- If you use a different Python installation, replace the paths to the DLLs with the matching `Library/bin` in your Python/conda install.

Troubleshooting
- FileNotFoundError for `behaviors` or `sensors`: make sure both are included via `--add-data`. While debugging, use `--onedir` instead of `--onefile` so you can inspect extracted files.
- Missing native DLL / ImportError (e.g. _ctypes, libcrypto): check `build\mission-constructor\warn-mission-constructor.txt` for missing libraries and add them with `--add-binary "<path-to-dll>;."`.
- PermissionError when PyInstaller tries to remove `build\mission-constructor\localpycs`: remove the directory manually and re-run:

```powershell
Remove-Item -LiteralPath "build\\mission-constructor\\localpycs" -Recurse -Force -ErrorAction Ignore
```

- If the EXE shows a runtime error referencing `C:\\msys64\\tmp\\_MEI...\\sensors` and it says "Permission denied" or "No such file or directory":
  - Confirm `--add-data './sensors;sensors'` was used.
  - For debugging builds, use `--onedir` to inspect the extracted `sensors` target.
  - The code includes `repo_root()` handling of `sys._MEIPASS` so files should resolve inside the bundle.

Useful commands
- Rebuild with onedir (easier to debug extracted files):

```powershell
.":\"Review\\.venv\\Scripts\\python.exe" -m PyInstaller --clean --onedir --windowed --name mission-constructor --add-data "./behaviors;." --add-data "./sensors;sensors" SUtilities/launch_mission_constructor.py
```

- Run the final exe (from repo root):

```powershell
.\dist\mission-constructor.exe
```

Where to look for logs
- PyInstaller warnings: `build\mission-constructor\warn-mission-constructor.txt`
- Graph xref: `build\mission-constructor\xref-mission-constructor.html`

Repository notes
- `SUtilities/mission_model.py` was made tolerant to `behaviors`/`sensors` being files or directories and detects `sys._MEIPASS` when running from a PyInstaller bundle.

If something fails, copy the full PyInstaller `warn-*.txt` and the exact runtime traceback and I can advise the minimal `--add-binary` or code fix.
