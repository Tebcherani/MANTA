# Makefile — Odyssey AUV Standalone Simulator
# Target: Linux / GCC
# Build: make
# Run:   ./ody sim <mission_name> 

CC       = gcc
CPPFLAGS = -I.              \
    -Isrc/Application       \
    -Isrc/Common            \
    -Isrc/Odyssey           \
    -Isrc/Odyssey/Behaviors \
    -Isrc/Simulation        \
    -Isrc/World
CFLAGS   = -O2 -Wall -Wno-missing-braces -g -std=gnu89
LDFLAGS  = -lm

# Machine-specific overrides belong outside the OneDrive workspace.
# Example Windows path:
#   C:/Users/<you>/.ody-auv-sim/Windows_Ody_C.mk
# Put CC, CFLAGS, or PATH-sensitive tool overrides there.
USERPROFILE_POSIX := $(subst \,/,$(USERPROFILE))
-include $(HOME)/.ody-auv-sim/Windows_Ody_C.mk
-include $(USERPROFILE_POSIX)/.ody-auv-sim/Windows_Ody_C.mk

TARGET  = sim
EXEEXT =
SIM_EXE = ./$(TARGET)$(EXEEXT)
MISSION ?= dive240.mi

SRCS = \
    src/Application/command_line.c                      \
    src/Common/angle_utils.c                            \
    src/Common/random_utils.c                           \
    src/Odyssey/Behaviors/odyssey_behaviors_wiggle.c    \
    src/Odyssey/odyssey_config.c                        \
    src/Odyssey/odyssey_control.c                       \
    src/Odyssey/odyssey_data.c                          \
    src/Odyssey/odyssey_data_logger.c                   \
    src/Odyssey/odyssey_log.c                           \
    src/Odyssey/odyssey_mission.c                       \
    src/Odyssey/odyssey_model.c                         \
    src/Odyssey/odyssey_nav.c                           \
    src/Odyssey/odyssey_sensors.c                       \
    src/Odyssey/odyssey_sim.c                           \
    src/Simulation/sim_behavior_logger.c                \
    src/Simulation/sim_config.c                         \
    src/Simulation/sim_exit.c                           \
    src/Simulation/sim_log.c                            \
    src/Simulation/sim_orchestrator.c                   \
    src/Simulation/sim_state.c                          \
    sim.c               \
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
    diagnostic.c        \
    lbl.c               \
    lbl_nav.c           \
    waypoint.c          \
    whoi_homing.c       \
    edc_homing.c        \
    docking.c           \
    sim_stubs.c         \
    load_fault.c        \
    odyssey_legacy_exit.c 

OBJS = $(SRCS:.c=.o)

.PHONY: all clean setup debug both edit_struct dev-setup doctor run-sim review review2 mission-constructor mission-constructor-web

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

edit_struct: edit_struct.c
	$(CC) -o edit_struct edit_struct.c

dev-setup:
	./ody setup

doctor:
	./ody doctor

run-sim: $(TARGET)
	$(SIM_EXE) $(MISSION)

review:
	./ody review

review2:
	./ody review2

mission-constructor:
	./ody mission-constructor

mission-constructor-web:
	./ody mission-constructor-web

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET) sim_debug edit_struct

debug: CFLAGS += -fsanitize=address
debug: clean $(TARGET)
	mv $(TARGET) sim_debug

# Runtime file dependencies:
#   ./get_am.tex    — hydrodynamic added-mass coefficients (read by sim.c)
#   ./get_drag.tex  — drag coefficients (read by sim.c)
#   ./get_shape.tex — vehicle shape parameters (read by sim.c)