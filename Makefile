# Makefile — Odyssey AUV Standalone Simulator
# Target: Linux / GCC
# Build: make
# Run:   ./ody sim <mission_name>   (bare names are resolved in ./SMission/)

CC      = gcc
CFLAGS  = -O2 -Wall -Wno-missing-braces -g -std=gnu89 -I.
LDFLAGS = -lm

# Machine-specific overrides belong outside the OneDrive workspace.
# Example Windows path:
#   C:/Users/<you>/.ody-auv-sim/Windows_Ody_C.mk
# Put CC, CFLAGS, or PATH-sensitive tool overrides there.
USERPROFILE_POSIX := $(subst \,/,$(USERPROFILE))
-include $(HOME)/.ody-auv-sim/Windows_Ody_C.mk
-include $(USERPROFILE_POSIX)/.ody-auv-sim/Windows_Ody_C.mk

TARGET  = sim
# On Linux, ksim/ is both the source directory and holds ksim/ksim.cfg at runtime,
# so the binary cannot be named 'ksim' (linker error: is a directory).
# On Windows, MinGW gcc appends .exe automatically so 'ksim' -> 'ksim.exe'.
ifeq ($(OS),Windows_NT)
    EXEEXT = .exe
    KSIM_TARGET = ksim
else
    EXEEXT =
    KSIM_TARGET = ksim_bin
endif
SIM_EXE = ./$(TARGET)$(EXEEXT)
KSIM_EXE = ./$(KSIM_TARGET)$(EXEEXT)
MISSION ?= dive240.mi
KSIM_BEHAVIOR ?= SUtilities/ksim_runs/latest_behavior.json

SRCS = \
    sim_main.c          \
    sim.c               \
    sim_bottom.c        \
    sim_current.c       \
    sim_conduct.c       \
    sim_usbl.c          \
    sensor_load.c       \
    layered_control.c   \
    dynamic_control.c   \
    whoi_traj.c         \
    log_data.c          \
    load_mission.c      \
    paths.c             \
    vehicle.c           \
    update.c            \
    diagnostic.c        \
    fileman.c           \
    filters.c           \
    nrutil.c            \
    lbl.c               \
    lbl_nav.c           \
    waypoint.c          \
    wiggle.c            \
    whoi_homing.c       \
    edc_homing.c        \
    docking.c           \
    getopt.c            \
    sim_stubs.c         \
    load_fault.c

KSIM_SRCS = $(filter-out sim.c,$(SRCS)) \
    ksim/sim_kinematic.c \
    ksim/ksim_stubs.c

# Files intentionally excluded and why:
#   main.c              — real-vehicle entry point; replaced by sim_main.c
#   yoyo_new.c          — file begins with email headers; broken C
#   yo_yo_depth.c       — missing includes and broken syntax; excluded pending fix
#   yo_yo_temperature.c — same as above
#   ekf_work.c          — requires matrix library (libody_dev.a) not present
#   sensor_kvh.c        — same matrix library dependency; heading_reference() stubbed
#   serial.c            — hardware serial bus; unreachable in sim; stubs in sim_stubs.c
#   ody_serio.c         — same; stubs in sim_stubs.c
#   wfparse.c           — standalone post-processing utility with its own main()
#   io_utils.c          — contains only hardware I/O functions not needed by the simulator

OBJS = $(SRCS:.c=.o)
KSIM_OBJS = $(KSIM_SRCS:.c=.o)

.PHONY: all clean setup debug both edit_struct dev-setup doctor run-sim run-ksim review review2 mission-constructor mission-constructor-web

all: setup $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(KSIM_TARGET): $(KSIM_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

both: setup $(TARGET) $(KSIM_TARGET)

edit_struct: edit_struct.c
	$(CC) -o edit_struct edit_struct.c

.PHONY: ksim-test ksim-run-test
ksim-test: setup $(KSIM_TARGET)
	$(KSIM_EXE) --parse $(MISSION)

ksim-run-test: setup $(KSIM_TARGET)
	$(KSIM_EXE) --nolog --dump-behavior behavior_log.json $(MISSION)
	@test -f behavior_log.json

dev-setup:
	./ody setup

doctor:
	./ody doctor

run-sim: setup $(TARGET)
	$(SIM_EXE) $(MISSION)

run-ksim: setup $(KSIM_TARGET)
	@mkdir -p SUtilities/ksim_runs
	$(KSIM_EXE) --nolog --dump-behavior $(KSIM_BEHAVIOR) $(MISSION)

review:
	./ody review

review2:
	./ody review2

mission-constructor:
	./ody mission-constructor

mission-constructor-web:
	./ody mission-constructor-web

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

setup:
	@mkdir -p SData SMission SFault

clean:
	rm -f $(OBJS) $(KSIM_OBJS) $(TARGET) $(KSIM_TARGET) sim_debug edit_struct

debug: CFLAGS += -fsanitize=address
debug: clean $(TARGET)
	mv $(TARGET) sim_debug

# Runtime file dependencies:
#   ./get_am.tex    — hydrodynamic added-mass coefficients (read by sim.c)
#   ./get_drag.tex  — drag coefficients (read by sim.c)
#   ./get_shape.tex — vehicle shape parameters (read by sim.c)
#   ./SMission/     — mission script files (.mi)
#   ./SData/        — data output directory (created by 'make setup')
#   ./system.log    — runtime diagnostic log
