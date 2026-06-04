# dist

This folder contains packaged build outputs for the mission constructor.

## Files

- `mission-constructor.exe`
  - Single-file Windows executable built with PyInstaller.
  - Includes bundled mission constructor GUI, behavior data, sensor data, and required native libraries.

- `BUILD_MISSION_CONSTRUCTOR.md`
  - Reproducible build instructions for rebuilding `mission-constructor.exe` from source.

## How to use

1. Open a terminal in this repo root.
2. Run:

```powershell
.\dist\mission-constructor.exe
```

## Notes

- `mission-constructor.exe` is a generated artifact and is large because it contains the full Python runtime and data.
- If you need to rebuild it, follow the steps in `BUILD_MISSION_CONSTRUCTOR.md`.
