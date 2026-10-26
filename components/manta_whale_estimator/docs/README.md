# MANTA whale estimator

This component is a Python ROS 2 node per vehicle. Odyssey triangulates paired bearings and publishes a timestamped bearing-position measurement with north/east variances. The node uses FilterPy to update the position and velocity estimate, then publishes it back to Odyssey for guidance between measurements. The motion model returns velocity gradually toward its configured baseline. Once three recent measurements are available, a short rolling fit supplies the velocity estimate.

`ros/whale_estimator/src/` contains the estimator's Python package files. `whale_estimator_state.py` holds the per-node filter state; `whale_estimator_state_log.py` builds snapshots and writes change-only JSON logs; `whale_estimator_types.py` defines the reusable configuration and bearing-position measurement dataclasses. `filter.py` contains the mean-reverting model, FilterPy predict/update calls, and rolling fit. `whale_estimator_node.py` handles ROS subscriptions, publication, and configuration. Duplicate or older timestamps are rejected. Each measurement's variance is used as the measurement variance `R`.

`docs` contains PlantUML diagrams for initialization and the update/prediction sequence.

The ROS package is in `ros/whale_estimator`. Its tuning lives in `ros/whale_estimator/config.yaml`; the trailing mission starts one node per vehicle and supplies only `vehicle_id` at runtime. Each node subscribes to `/<vehicle_id>/whale/bearing_position_estimate` and publishes `/<vehicle_id>/whale/kalman_estimate`. The native Odyssey simulator no longer links or runs the estimator model.

`WhaleBearingPositionEstimate` is defined in Odyssey's `odyssey_interfaces` package. `WhaleKalmanEstimate` is defined in the sibling `ros/whale_estimator_interfaces` package.

The development image installs FilterPy from `ros/whale_estimator/requirements.txt` plus NumPy and SciPy. Build the ROS package through the normal MANTA build command.
