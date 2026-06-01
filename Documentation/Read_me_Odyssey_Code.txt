There are two papers attached here come up the task fit configuration with layered control paper is by far the most useful for the existing code base. 

The thing to understand is that the code is single threaded and sequences between reading sensors from real subsystems, replaced by simulator calls when you're running the simulator, processing sensory data, managing the dynamic control of the vehicle, sequencing & specifying mission activities, and logging data.

The heart of the vehicle software is main.c.  It loads missions via the load_mission function and sets up the vehicle data structure (more on this below).   It initializes key subsystems (e.g. the serial bus), sensors (most have unit tests) and actuators.  It provides user feedback on status, provides an operator countdown, and a final pre-dive arm.

The main control loop runs at a 5Hz rate, and 

io() —handles all serial port reads/writes for each sensor 
sensor_processing() — navigation filtering, unit conversion, dead reckoning 
dynamic_control() — PID control law execution 
layered_control() — behavior arbitration and mission state machine
Log_data() —  logs variables which have been updated

At the end of the mission, the vehicle executes exit_gracefully().

The data structure:  The vehicle is organized around the data structure, specified in the header file, which the vehicle function calls of various types write to and read from. This might sound clunky, but it makes it easy to add new variables and therefore new behaviors and sensory functionality to the vehicle without having to worry about the mission interface, logging, or producing usable data files after a mission. All of those housekeeping functions are automatically handled for the data structure, so adding a new variable to the data structure means that that functionality is immediately applied to the new variable.

The variable structure is defined in a program edit_struct.c.  To modify the vehicle data structure one 

1) Edits the data structure definition containing in the first part of the edit_struct.c code, 
2) Compiles and runs edit_struct.c which generates new header and c files with the new data structure definition, and then 
3) Recompiles the vehicle code with the new header files.  

The one external file that edit_struct.c needs is a configuration file, which is a (very short) file that tells edit_struct which subsystems are present in the vehicle for which an executable will ultimately be generated.  This allows one to support multiple versions of vehicles with the same code base.  So the configuration files are usually names of specific vehicles, e.g. “Xanthos.cfg”.  

What edit struct actually does is to read its own source code for the data structure definition (new sensors, new behaviors) so the new source code for edit struct has to be in a place where it can be found when the new edit_struct is executed.  NOTE:  this is one of many places where the code could be improved, but being self-contained and consistent across users was important, and this keeps all the essential information in one file and automates the propagation of the new configuration, at the cost of having to recompile the entire vehicle code (which these days is pretty fast).

Adding a new variable:  The way one adds a new variable to the data structure, is:

1) Edit the edit_struct.c file, adding the new variable to the list of sensors along with units and a default (starting value), e.g.:
a. “sensor:  m_load_voltage(volt)” already exists.
b. “sensor:  m_battery_low_level_warning(Boolean)”  might be a variable added to support a routine that has higher fidelity understanding of battery state.
2) Run the new edit_struct.  This will generate new header files with the new variable in them.  
3) Then one recompiles the code. The recompilation process is necessary for all of the code to know about the new sensor.

Variable naming convention:  Variables have names like m_roll_rate. u_cutoff_voltage, uo_control_pitch_lim, c_heading, and dgc_calcount.  The prefix is a notation to help programmers and vehicle operators understand if the variable is updated internally (m_), expected to be set by the user (u_), an important user configuration variable (uo_), a variable used by the simulator for tracking simulator state (real_ and maybe also sim_), or a variable associated with a specific subsystem (e.g. rdi_range1 is the range on beam one of the RDI DVL, if installed).  

Variable units specification:    Programmers are strongly encouraged to specify the units when adding the variables.  The code is mks, radians, and other unitless parameters like ‘boolean’, counts, and %.  The only place where these unit conventions are violated is in mission files, where users can specify a variable value in degrees and the parser will convert degrees to radians.  However specifying the expected unit in edit_struct.c provides documentation of what is expected, and could come in very handy if unit problems occur.  

Updating a variable in the code:  Edit_struct.c creates an all upper case constant that is used to index into the data_structure to read the variable state.  So, to access the proportional term of the heading control loop, “uo_control_kph” one would use the following:

Kph = sensor( UO_CONTROL_KPH );

However, to write to a variable, one uses a function call update.  The reason that using this function call is important is that update also modifies the time stamp on the variable, thus signaling to other software elements (e.g. critically the logging routine) that there is a new value written to the variable.  So to modify “uo_control_kph” to take on the value of “new_kph”, one would simply write:

Update(UO_CONTROL_KPH, new_kph);

Adding a new behavior:  Missions are configured by loading a behavior stack with behaviors and configuring the arguments of the behaviors.  To add a new behavior, one adds it as a function call in the layered_control.com file.  To be able to use the behavior, one must add it to the vehicle data structure by adding the behavior name and argument list to edit_struct.c and then follow the procedure above of a) compiling and running edit_struct, b) recompiling the vehicle code.  Here is an example of a behavior definition:

behavior: acquire_heading
  b_arg:           heading(rad) 0.0
  b_arg:          acceptable_error(rad) 0.0
  b_arg:          behavior_active(bool) 0.0

Behaviors can have up to ten behavior arguments (MAX_B_ARG as defined in ody_defs.h).  Once the behavior has been written and added to layered_control.c, and the behavior name and behavior argument definitions have been added via the edit_struct process, one can use the behavior in mission scripts without other code modification.  

Configuring a mission:  Missions are simply text files that configure the data structure.  The data structure is configured by specifying values for sensors, activating behaviors by identifying them by name, and loading behavior arguments.  The way that these layered control stack (also called behavior stacks) accomplish missions is described in “Bellingham, James G., and J. Leonard. "Task configuration with layered control." In IARP Workshop on Mobile Robots for Subsea Environments. 1994.”


