#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
VENV_PY="$REPO_ROOT/.venv/bin/python"

ok=0
warn=0
fail=0

pass() {
  printf 'ok    %s\n' "$1"
  ok=$((ok + 1))
}

note_warn() {
  printf 'warn  %s\n' "$1"
  warn=$((warn + 1))
}

note_fail() {
  printf 'fail  %s\n' "$1"
  fail=$((fail + 1))
}

check_cmd() {
  if command -v "$1" >/dev/null 2>&1; then
    pass "$1: $(command -v "$1")"
  else
    note_fail "$1 not found"
  fi
}

echo "Odyssey Ubuntu environment check"
echo "Repository: $REPO_ROOT"
echo

case "$REPO_ROOT" in
  /mnt/c/*|/mnt/d/*|/mnt/e/*)
    note_warn "repo is under /mnt/*; WSL builds are faster and more reliable under ~/src"
    ;;
  *)
    pass "repo is on the Linux filesystem"
    ;;
esac

check_cmd git
check_cmd make
check_cmd gcc
check_cmd python3

if [ -x "$VENV_PY" ]; then
  pass "virtual environment: $VENV_PY"
  if "$VENV_PY" -c 'import PyQt5, matplotlib, numpy, scipy, xarray, ody_review' >/dev/null 2>&1; then
    pass "Python imports: PyQt5 matplotlib numpy scipy xarray ody_review"
  else
    note_fail "Python dependencies are incomplete; run ./ody setup"
  fi
  qt_xcb="$("$VENV_PY" - <<'PY'
import glob
import sys

matches = glob.glob(
    sys.prefix + "/lib/python*/site-packages/PyQt5/Qt5/plugins/platforms/libqxcb.so"
)
print(matches[0] if matches else "")
PY
)"
  if [ -n "$qt_xcb" ] && command -v ldd >/dev/null 2>&1; then
    missing_qt_libs="$(ldd "$qt_xcb" | awk '/not found/ { print $1 }' | sort -u | tr '\n' ' ')"
    if [ -n "$missing_qt_libs" ]; then
      note_fail "Qt xcb plugin is missing native libraries: $missing_qt_libs"
      note_fail "install GUI libraries with: ./ody setup --system"
    else
      pass "Qt xcb native libraries resolved"
    fi
  elif [ -z "$qt_xcb" ]; then
    note_warn "Qt xcb plugin not found under .venv"
  else
    note_warn "ldd not found; cannot inspect Qt xcb native libraries"
  fi
else
  note_warn "virtual environment missing; run ./ody setup"
fi

if [ -x "$REPO_ROOT/sim" ]; then
  pass "sim executable exists"
else
  note_warn "sim executable missing; run ./ody build"
fi

if [ -x "$REPO_ROOT/ksim_bin" ]; then
  pass "ksim_bin executable exists"
else
  note_warn "ksim_bin executable missing; run ./ody build"
fi

if [ -n "${DISPLAY-}" ] || [ -n "${WAYLAND_DISPLAY-}" ]; then
  pass "GUI display is available"
else
  note_warn "no DISPLAY/WAYLAND_DISPLAY detected; GUI apps need WSLg, X11, or a desktop session"
fi

echo
printf 'Summary: %d ok, %d warning(s), %d failure(s)\n' "$ok" "$warn" "$fail"

if [ "$fail" -gt 0 ]; then
  exit 1
fi
