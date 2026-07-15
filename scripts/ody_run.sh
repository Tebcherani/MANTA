#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$REPO_ROOT/.venv"
VENV_PY="$VENV_DIR/bin/python"
VENV_BIN="$VENV_DIR/bin"
DEFAULT_MISSION="dive240.mi"

usage() {
  cat <<'EOF'
Usage: ./ody <command> [args]

Setup:
  ./ody setup [--system] [--no-build]   create/update .venv and build sim + ksim
  ./ody doctor                          check Ubuntu tools, venv, build outputs, GUI display
  ./ody build                           build sim and ksim_bin
  ./ody clean                           remove C build products

Run:
  ./ody sim [mission] [fault_file]       run ./sim, default mission is dive240.mi
  ./ody ksim [args]                     run ./ksim_bin, default is --parse dive240.mi
  ./ody review [args]                   launch Odyssey Review GUI
  ./ody review2 [args]                  launch Review2 compare GUI or CLI
  ./ody mission-constructor             launch PyQt mission constructor
  ./ody mission-constructor-web [args]  launch legacy web mission constructor

Short aliases:
  mc      mission-constructor
  mc-web  mission-constructor-web

Examples:
  ./ody setup --system
  ./ody sim B9801810.mi
  ./ody ksim --nolog --dump-behavior SUtilities/ksim_runs/latest.json dive240.mi
  ./ody review
  ./ody review2 SData/run1.obd SData/run2.obd --opts all --save Review/compare_out
EOF
}

ensure_venv() {
  if [ ! -x "$VENV_PY" ]; then
    "$REPO_ROOT/scripts/setup_venv.sh"
  fi
}

ensure_sim() {
  if [ ! -x "$REPO_ROOT/sim" ]; then
    make -C "$REPO_ROOT" sim
  fi
}

ensure_ksim() {
  if [ ! -x "$REPO_ROOT/ksim_bin" ]; then
    make -C "$REPO_ROOT" ksim_bin
  fi
}

if [ "$#" -eq 0 ]; then
  usage
  exit 0
fi

cmd="$1"
shift

case "$cmd" in
  help|-h|--help)
    usage
    ;;

  setup)
    install_system=0
    build=1
    while [ "$#" -gt 0 ]; do
      case "$1" in
        --system)
          install_system=1
          ;;
        --no-build)
          build=0
          ;;
        *)
          echo "Unknown setup option: $1" >&2
          exit 2
          ;;
      esac
      shift
    done
    if [ "$install_system" -eq 1 ]; then
      "$REPO_ROOT/scripts/setup_system_ubuntu.sh"
    fi
    "$REPO_ROOT/scripts/setup_venv.sh"
    if [ "$build" -eq 1 ]; then
      make -C "$REPO_ROOT" both
    fi
    "$REPO_ROOT/scripts/doctor_ubuntu.sh"
    ;;

  doctor)
    "$REPO_ROOT/scripts/doctor_ubuntu.sh"
    ;;

  build)
    make -C "$REPO_ROOT" both
    ;;

  clean)
    make -C "$REPO_ROOT" clean
    ;;

  sim)
    ensure_sim
    if [ "$#" -eq 0 ]; then
      set -- "$DEFAULT_MISSION"
    fi
    cd "$REPO_ROOT"
    exec ./sim "$@"
    ;;

  ksim)
    ensure_ksim
    if [ "$#" -eq 0 ]; then
      set -- --parse "$DEFAULT_MISSION"
    fi
    cd "$REPO_ROOT"
    exec ./ksim_bin "$@"
    ;;

  review)
    ensure_venv
    cd "$REPO_ROOT"
    exec "$VENV_BIN/ody-review" "$@"
    ;;

  review2)
    ensure_venv
    cd "$REPO_ROOT"
    exec "$VENV_BIN/review2" "$@"
    ;;

  mission-constructor|mc)
    ensure_venv
    cd "$REPO_ROOT"
    exec "$VENV_PY" SUtilities/launch_mission_constructor.py "$@"
    ;;

  mission-constructor-web|mc-web)
    ensure_venv
    ensure_ksim
    cd "$REPO_ROOT"
    exec "$VENV_PY" SUtilities/mission_constructor_app.py "$@"
    ;;

  *)
    echo "Unknown command: $cmd" >&2
    echo >&2
    usage >&2
    exit 2
    ;;
esac
