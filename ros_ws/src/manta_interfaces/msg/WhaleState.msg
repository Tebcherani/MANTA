# Unique identifier assigned by the scenario simulator.
string whale_id

# Shared MANTA simulation cycle represented by this state.
uint64 cycle

# Position in the MANTA north-east-down coordinate frame, in meters.
float64 north
float64 east
float64 depth

# Velocity in the north-east-down coordinate frame, in meters per second.
float64 velocity_north
float64 velocity_east
float64 velocity_down

# True after the whale's randomly assigned scenario start time.
bool active

# True when the whale emits a call during this simulation cycle.
bool calling