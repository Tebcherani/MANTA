# Code Structure

MANTA has four main folders:
1. components - to hold the core pieces of the simulator code
2. log - to hold simulator logs
3. missions - to hold code for each mission type
4. output - to hold outputs associated with mission runs

## components

Each component should have its own folder. That folder should contain two folders: docs and ros. 

docs should have an *_init.puml and *_sequence.puml that is always kept up-to-date with each commit to main. These should be high-level diagrams that contain only functionality that interact with the node itself.

ros should have an *_interfaces folder that contains all the ROS message and service types.

ros should also have a *_sim folder. That folder should contain an include and src folder. It should also contain a config.yaml file. 

The config.yaml file should contain state variables that need to be initialized, along with their initial values. 

The include folder should contain another *_sim folder, and then *_node.hpp, *_state_json.hpp, and *_state.hpp files. *_node.hpp is the node header file. *_state_json.hpp is a Codex-generated file for state logging. *_state.hpp should contain all state variables, and all variables that would be passed through functions. That is, no variable should be passed through a function - it should go through the state struct instead. Each variable should have a clear name with units at the end. Boolean variables get units of *_b.

At a minimum, the src folder will include a *_callbacks.cpp file which contains callbacks for the subscribers, an *_interfaces.cpp file which contains the functions that create ROS publishers, subscribers, etc., *_node.cpp, and *_setup.cpp, which contains setup functions. Note that automatic state logging should be called at the end of each subscription in *_interfaces.cpp using the functions in the *_state_json.hpp file.

If code is in Python, such as manta_whale_estimator, it should follow a similar structure. 