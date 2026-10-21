# MANTA whale estimator

This component is a ROS 2 node per vehicle. Odyssey triangulates paired bearings and publishes a timestamped position fix to the vehicle's estimator node. That node updates its filter and publishes the resulting estimate back to Odyssey. Odyssey uses the returned estimate for guidance predictions between fixes. The model uses mean-reverting velocity and process noise; once three recent fixes are available, a short rolling fit supplies the velocity estimate.

The headers in `ros/whale_estimator/include/whale_estimator` define the estimator types, state, and C model API. The ROS node stores one `whale_estimator_state_t`, sets `state.received_fix` for each message, and calls `whale_estimator_update`. Updates reject duplicate or older timestamps. Position fixes are treated as exact (`R = 0`).

`ros/whale_estimator/docs` contains PlantUML diagrams for initialization and the update/prediction sequence.

The ROS package is in `ros/whale_estimator`. Its tuning lives in `ros/whale_estimator/config.yaml`; the trailing mission starts one node per vehicle and supplies only `vehicle_id` at runtime. Each node subscribes to `/<vehicle_id>/whale/bearing_position_estimate` and publishes `/<vehicle_id>/whale/kalman_estimate`. The native Odyssey simulator no longer links or runs the estimator model.

`WhaleBearingPositionEstimate` is defined in Odyssey's `odyssey_interfaces` package. `WhaleKalmanEstimate` is defined in the sibling `ros/whale_estimator_interfaces` package.

Run the C model test with `python3 -m unittest discover -s ros/whale_estimator/tests` from this directory. Build the ROS package through the normal MANTA build command.
