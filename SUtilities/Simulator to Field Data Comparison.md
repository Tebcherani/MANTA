I have a lot of historical AUV runs.  In raw form, a vehicle run generates two files:  a *.slg file which contains the mission script at the top, and then various printf outputs from the vehicle while it was running.  The *.obd file contains the vehicle run data.  I want to take real vehicle missions and execute them in the simulator, comparing performance.  Ultimately, I will want to drive the simulated vehicle with control commands from the field runs to test the hydrodynamic modeling fidelity.  

Here are the goals:
1) I need a Python file 'extract_mission.py' that extracts the mission script from all the NAME.slg files in a folder, and stores them as NAME.mi
2) I need a second routine 'missing_mi_variables.py' that looks at all the NAME.mi in a folder, and identifies variables that are being used in the mission file that do not exist in the simulator data structure (vehicle.h).  It should do the same for behaviors.  It should turn this into a table in a cvs format, which lists which mission scripts have variables that are not in simulator data structure.
3) I need a third routine 'missing_sim_variables.py' that looks at the list of variables in the front of the *.obd (e.g. the data structure for the vehicle field data) and identifies variables that are in the field obd that are not in the simulator data structure.
4) I need a fourth routine 'trim_mi' that scans the mi files in a folder, and takes a 'NAME.mi' file and generates a new file 'NAME_trim.mi' that has variables not in the simulator data structure removed.

All these should take the folder to be processed as an argument.

The data structure for the AUV can be found in /c/Users/jbellin1/OneDrive - Johns Hopkins/Matlab-JHU/AUV_Mission_Sim/AUV_Sim/Windows_Ody_C