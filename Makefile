# Makefile — Odyssey AUV Standalone Simulator
# Target: Linux / GCC
# Build:  make
# Run:    ./ody sim <mission_name>

CC = gcc
AR = ar

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

ODYSSEY_CPPFLAGS = \
	$(COMMON_CPPFLAGS) \
	-Isrc/Odyssey \
	-Isrc/Odyssey/Behaviors \
	-Isrc/Odyssey/Control \
	-Isrc/Odyssey/Faults \
	-Isrc/Odyssey/Navigation \
	-Isrc/Odyssey/Sensors

CFLAGS = -O2 -Wall -Wno-missing-braces -g -std=gnu89
LDFLAGS = -llapacke -llapack -lblas -lm

TARGET = sim
EXEEXT =
SIM_EXE = ./$(TARGET)$(EXEEXT)

MISSION ?= dive240.mi

BUILD_DIR = build

COMMON_SRCS = \
	src/Common/angle_utils.c \
	src/Common/matrix_utils.c \
	src/Common/random_utils.c \
	src/Common/sim_log.c \
	src/Common/unit_utils.c

SIM_SRCS = \
	src/Application/command_line.c \
	src/Simulation/sim_config.c \
	src/Simulation/sim_exit.c \
	src/Simulation/sim_orchestrator.c \
	src/Simulation/sim_state.c

WORLD_SRCS = \
	src/World/world_config.c \
	src/World/world_sim.c

ODYSSEY_SRCS = \
	src/Odyssey/Behaviors/edc_homing.c \
	src/Odyssey/Behaviors/odyssey_behavior_registry.c \
	src/Odyssey/Behaviors/odyssey_behaviors_wiggle.c \
	src/Odyssey/Behaviors/odyssey_behaviors_dock.c \
	src/Odyssey/Behaviors/waypoint.c \
	src/Odyssey/Behaviors/whoi_homing.c \
	src/Odyssey/Control/dynamic_control.c \
	src/Odyssey/Control/layered_control.c \
	src/Odyssey/Control/odyssey_control.c \
	src/Odyssey/Control/whoi_traj.c \
	src/Odyssey/Faults/load_fault.c \
	src/Odyssey/Faults/paths.c \
	src/Odyssey/odyssey_behavior_logger.c \
	src/Odyssey/odyssey_data_logger.c \
	src/Odyssey/odyssey_data_struct.c \
	src/Odyssey/odyssey_mission.c \
	src/Odyssey/odyssey_model.c \
	src/Odyssey/sensor_load.c \
	src/Odyssey/sim_stubs.c \
	src/Odyssey/odyssey_sim.c \
	src/Odyssey/sim.c \
	src/Odyssey/vehicle.c \
	src/Odyssey/Navigation/odyssey_nav.c \
	src/Odyssey/Navigation/odyssey_nav_dead_reckoning.c \
	src/Odyssey/Navigation/odyssey_nav_lbl.c \
	src/Odyssey/Sensors/odyssey_sensors.c \
	src/Odyssey/Sensors/odyssey_sensors_ahrs.c \
	src/Odyssey/Sensors/odyssey_sensors_depth.c \
	src/Odyssey/Sensors/odyssey_sensors_lbl.c \
	src/Odyssey/Sensors/odyssey_sensors_usbl.c \
	src/Odyssey/Sensors/odyssey_sensors_water.c \
	src/Odyssey/Sensors/sim_conduct.c

COMMON_OBJS = $(COMMON_SRCS:%.c=$(BUILD_DIR)/%.o)
SIM_OBJS = $(SIM_SRCS:%.c=$(BUILD_DIR)/%.o)
WORLD_OBJS = $(WORLD_SRCS:%.c=$(BUILD_DIR)/%.o)
ODYSSEY_OBJS = $(ODYSSEY_SRCS:%.c=$(BUILD_DIR)/%.o)

ODYSSEY_LIB = libodyssey.a

EDIT_STRUCT = tools/edit_struct
EDIT_STRUCT_SRC = tools/edit_struct.c

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

all: $(TARGET)

$(ODYSSEY_LIB): $(ODYSSEY_OBJS)
	$(AR) rcs $@ $^

$(TARGET): $(SIM_OBJS) $(WORLD_OBJS) $(COMMON_OBJS) $(ODYSSEY_LIB)
	$(CC) $(CFLAGS) -o $@ \
		$(SIM_OBJS) \
		$(WORLD_OBJS) \
		$(COMMON_OBJS) \
		$(ODYSSEY_LIB) \
		$(LDFLAGS)

edit_struct: $(EDIT_STRUCT)

$(EDIT_STRUCT): $(EDIT_STRUCT_SRC)
	$(CC) $(ODYSSEY_CPPFLAGS) $(CFLAGS) \
		-o $(EDIT_STRUCT) \
		$(EDIT_STRUCT_SRC)

generate_odyssey: $(EDIT_STRUCT)
	./$(EDIT_STRUCT)

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

$(BUILD_DIR)/src/Simulation/%.o: src/Simulation/%.c
	@mkdir -p $(dir $@)
	$(CC) $(SIM_CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/src/Application/%.o: src/Application/%.c
	@mkdir -p $(dir $@)
	$(CC) $(SIM_CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/src/World/%.o: src/World/%.c
	@mkdir -p $(dir $@)
	$(CC) $(WORLD_CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/src/Odyssey/%.o: src/Odyssey/%.c
	@mkdir -p $(dir $@)
	$(CC) $(ODYSSEY_CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/src/Common/%.o: src/Common/%.c
	@mkdir -p $(dir $@)
	$(CC) $(COMMON_CPPFLAGS) $(CFLAGS) -c -o $@ $<

clean:
	rm -rf $(BUILD_DIR)
	rm -f \
		$(ODYSSEY_LIB) \
		$(TARGET) \
		sim_debug \
		$(EDIT_STRUCT)

debug: CFLAGS += -fsanitize=address
debug: clean $(TARGET)
	mv $(TARGET) sim_debug