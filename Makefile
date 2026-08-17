# Makefile — Odyssey AUV Standalone Simulator
# Target: Linux / GCC
# Build:  make
# Run:    make run-sim MISSION=dive240.mi
#
# Simulation output is written outside the source tree.

CC = gcc
AR = ar

# -------------------------------------------------------------------------
# Compiler flags
# -------------------------------------------------------------------------

COMMON_CPPFLAGS = \
	-I. \
	-Iinclude \
	-Isrc/Common

SIM_CPPFLAGS = \
	$(COMMON_CPPFLAGS) \
	-Isrc/Application \
	-Isrc/Simulation \
	-Isrc/World

WORLD_CPPFLAGS = \
	$(COMMON_CPPFLAGS) \
	-Isrc/World

SENSOR_CPPFLAGS = \
	$(COMMON_CPPFLAGS) \
	-Isrc/Sensors

ODYSSEY_CPPFLAGS = \
	$(COMMON_CPPFLAGS) \
	-Isrc/Sensors \
	-Isrc/Odyssey \
	-Isrc/Odyssey/Vehicle \
	-Isrc/Odyssey/Simulator \
	-Isrc/Odyssey/Simulator/Faults \
	-Isrc/Odyssey/Simulator/Sensors \
	-Isrc/Odyssey/Vehicle/Behaviors \
	-Isrc/Odyssey/Vehicle/Control \
	-Isrc/Odyssey/Vehicle/Loggers \
	-Isrc/Odyssey/Vehicle/Navigation

CFLAGS = -O2 -Wall -Wno-missing-braces -g -std=gnu89
LDFLAGS = -llapacke -llapack -lblas -lm

# -------------------------------------------------------------------------
# Executable
# -------------------------------------------------------------------------

TARGET = sim
EXEEXT =

# Absolute path to the repository root.
PROJECT_DIR := $(abspath .)

# Absolute path to the simulator executable.
SIM_EXE := $(PROJECT_DIR)/$(TARGET)$(EXEEXT)

MISSION ?= dive240.mi

BUILD_DIR = build

# -------------------------------------------------------------------------
# Runtime output
# -------------------------------------------------------------------------
#
# Keep generated simulator output outside the source repository.
#
# Override from the command line if desired:
#
#   make run-sim RUN_DIR=/data/odyssey/runs
#
RUN_DIR ?= $(abspath ../odyssey-runs)

# -------------------------------------------------------------------------
# Common sources
# -------------------------------------------------------------------------

COMMON_SRCS = \
	src/Common/angle_utils.c \
	src/Common/matrix_utils.c \
	src/Common/random_utils.c \
	src/Common/sim_log.c \
	src/Common/time_utils.c \
	src/Common/unit_utils.c

# -------------------------------------------------------------------------
# Generic reusable sensor simulation sources
# -------------------------------------------------------------------------
#
# These sensor models should not depend on Odyssey-specific sensor IDs
# or Odyssey vehicle logic. They are intended to be reusable by other
# simulated vehicles in the future.
#

SENSOR_SRCS = \
	src/Sensors/sensor_compass.c \
	src/Sensors/sensor_depth.c \
	src/Sensors/sensor_water_conductivity.c \
	src/Sensors/sensor_water_speed.c \
	src/Sensors/sensor_water_temp.c

# -------------------------------------------------------------------------
# Simulation application sources
# -------------------------------------------------------------------------

SIM_SRCS = \
	src/Application/command_line.c \
	src/Simulation/sim_config.c \
	src/Simulation/sim_exit.c \
	src/Simulation/sim_orchestrator.c \
	src/Simulation/sim_state.c

# -------------------------------------------------------------------------
# World simulator sources
# -------------------------------------------------------------------------

WORLD_SRCS = \
	src/World/world_config.c \
	src/World/world_sim.c

# -------------------------------------------------------------------------
# Odyssey sources
# -------------------------------------------------------------------------

ODYSSEY_SRCS = \
	src/Odyssey/Vehicle/odyssey.c \
	src/Odyssey/Vehicle/odyssey_data_struct.c \
	src/Odyssey/Vehicle/odyssey_mission.c \
	src/Odyssey/Vehicle/vehicle.c \
	src/Odyssey/Vehicle/Behaviors/odyssey_behavior_registry.c \
	src/Odyssey/Vehicle/Behaviors/odyssey_behaviors_wiggle.c \
	src/Odyssey/Vehicle/Behaviors/odyssey_behaviors_dock.c \
	src/Odyssey/Vehicle/Behaviors/waypoint.c \
	src/Odyssey/Vehicle/Behaviors/whoi_homing.c \
	src/Odyssey/Vehicle/Control/dynamic_control.c \
	src/Odyssey/Vehicle/Control/layered_control.c \
	src/Odyssey/Vehicle/Control/odyssey_control.c \
	src/Odyssey/Vehicle/Control/whoi_traj.c \
	src/Odyssey/Vehicle/Loggers/odyssey_behavior_logger.c \
	src/Odyssey/Vehicle/Loggers/odyssey_data_logger.c \
	src/Odyssey/Vehicle/Navigation/odyssey_nav.c \
	src/Odyssey/Vehicle/Navigation/odyssey_nav_dead_reckoning.c \
	src/Odyssey/Vehicle/Navigation/odyssey_nav_lbl.c \
	src/Odyssey/Simulator/odyssey_model.c \
	src/Odyssey/Simulator/odyssey_sim.c \
	src/Odyssey/Simulator/sim.c \
	src/Odyssey/Simulator/Faults/load_fault.c \
	src/Odyssey/Simulator/Faults/paths.c \
	src/Odyssey/Simulator/Sensors/odyssey_sensors.c \
	src/Odyssey/Simulator/Sensors/odyssey_sensors_ahrs.c \
	src/Odyssey/Simulator/Sensors/odyssey_sensors_depth.c \
	src/Odyssey/Simulator/Sensors/odyssey_sensors_lbl.c \
	src/Odyssey/Simulator/Sensors/odyssey_sensors_usbl.c \
	src/Odyssey/Simulator/Sensors/odyssey_sensors_water.c

# -------------------------------------------------------------------------
# Object files
# -------------------------------------------------------------------------

COMMON_OBJS = $(COMMON_SRCS:%.c=$(BUILD_DIR)/%.o)
SENSOR_OBJS = $(SENSOR_SRCS:%.c=$(BUILD_DIR)/%.o)
SIM_OBJS = $(SIM_SRCS:%.c=$(BUILD_DIR)/%.o)
WORLD_OBJS = $(WORLD_SRCS:%.c=$(BUILD_DIR)/%.o)
ODYSSEY_OBJS = $(ODYSSEY_SRCS:%.c=$(BUILD_DIR)/%.o)

# -------------------------------------------------------------------------
# Odyssey static library
# -------------------------------------------------------------------------

ODYSSEY_LIB = libodyssey.a

# -------------------------------------------------------------------------
# edit_struct
# -------------------------------------------------------------------------

EDIT_STRUCT = tools/edit_struct
EDIT_STRUCT_SRC = tools/edit_struct.c

# -------------------------------------------------------------------------
# Phony targets
# -------------------------------------------------------------------------

.PHONY: \
	all \
	clean \
	debug \
	edit_struct \
	generate_odyssey \
	dev-setup \
	doctor \
	run-sim \
	review \
	review2 \
	mission-constructor \
	mission-constructor-web

# -------------------------------------------------------------------------
# Default build
# -------------------------------------------------------------------------

all: $(TARGET)

# -------------------------------------------------------------------------
# Odyssey library
# -------------------------------------------------------------------------

$(ODYSSEY_LIB): $(ODYSSEY_OBJS)
	$(AR) rcs $@ $^

# -------------------------------------------------------------------------
# Simulator executable
# -------------------------------------------------------------------------

$(TARGET): \
	$(SIM_OBJS) \
	$(WORLD_OBJS) \
	$(SENSOR_OBJS) \
	$(COMMON_OBJS) \
	$(ODYSSEY_LIB)

	$(CC) $(CFLAGS) -o $@ \
		$(SIM_OBJS) \
		$(WORLD_OBJS) \
		$(SENSOR_OBJS) \
		$(COMMON_OBJS) \
		$(ODYSSEY_LIB) \
		$(LDFLAGS)

# -------------------------------------------------------------------------
# edit_struct utility
# -------------------------------------------------------------------------

edit_struct: $(EDIT_STRUCT)

$(EDIT_STRUCT): $(EDIT_STRUCT_SRC)
	$(CC) $(ODYSSEY_CPPFLAGS) $(CFLAGS) \
		-o $(EDIT_STRUCT) \
		$(EDIT_STRUCT_SRC)

generate_odyssey: $(EDIT_STRUCT)
	./$(EDIT_STRUCT)

# -------------------------------------------------------------------------
# Development helpers
# -------------------------------------------------------------------------

dev-setup:
	./ody setup

doctor:
	./ody doctor

# -------------------------------------------------------------------------
# Run simulator
# -------------------------------------------------------------------------
#
# The simulator is launched with RUN_DIR as its working directory.
# Therefore files/directories created with relative paths, including .obd
# output, are placed there rather than inside the source repository.
#

run-sim: $(TARGET)
	@mkdir -p "$(RUN_DIR)"
	@echo "Simulation output directory: $(RUN_DIR)"
	cd "$(RUN_DIR)" && "$(SIM_EXE)" "$(MISSION)"

# -------------------------------------------------------------------------
# Review tools
# -------------------------------------------------------------------------

review:
	./ody review

review2:
	./ody review2

# -------------------------------------------------------------------------
# Mission constructors
# -------------------------------------------------------------------------

mission-constructor:
	./ody mission-constructor

mission-constructor-web:
	./ody mission-constructor-web

# -------------------------------------------------------------------------
# Compile rules
# -------------------------------------------------------------------------

$(BUILD_DIR)/src/Simulation/%.o: src/Simulation/%.c
	@mkdir -p $(dir $@)
	$(CC) $(SIM_CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/src/Application/%.o: src/Application/%.c
	@mkdir -p $(dir $@)
	$(CC) $(SIM_CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/src/World/%.o: src/World/%.c
	@mkdir -p $(dir $@)
	$(CC) $(WORLD_CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/src/Sensors/%.o: src/Sensors/%.c
	@mkdir -p $(dir $@)
	$(CC) $(SENSOR_CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/src/Odyssey/%.o: src/Odyssey/%.c
	@mkdir -p $(dir $@)
	$(CC) $(ODYSSEY_CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/src/Common/%.o: src/Common/%.c
	@mkdir -p $(dir $@)
	$(CC) $(COMMON_CPPFLAGS) $(CFLAGS) -c -o $@ $<

# -------------------------------------------------------------------------
# Clean
# -------------------------------------------------------------------------

clean:
	rm -rf $(BUILD_DIR)
	rm -f \
		$(ODYSSEY_LIB) \
		$(TARGET) \
		sim_debug \
		$(EDIT_STRUCT)

# -------------------------------------------------------------------------
# AddressSanitizer debug build
# -------------------------------------------------------------------------

debug: CFLAGS += -fsanitize=address
debug: clean $(TARGET)
	mv $(TARGET) sim_debug