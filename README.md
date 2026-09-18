# Drone Sim

- **[flight_sim](flight_sim/README.md)** — new native Mac flight simulator. RadioMaster Pocket USB input, Betaflight, and a 3D flying field. Start with `flight_sim/Launch Flight Sim.command`.
- **[matlab_sim](matlab_sim/README.md)** — the original MATLAB geometry analysis and scripted 6DOF simulation. In MATLAB, change the current folder to `matlab_sim`, then run `run_all_simulations`.

Choose **GoPro Drone** (the original deadcat model) or **DJI FPV** in the flight window. DJI FPV is an explicitly approximate aircraft model using published dimensions and mass, with estimated dynamics and Betaflight Acro controls. See [aircraft model sources and assumptions](flight_sim/docs/AIRCRAFT_MODELS.md).

On this Mac, the local Godot and Betaflight dependencies are included in the working folder, so the launcher is ready to use. Git excludes those generated/downloaded files. After cloning on another Mac, run `flight_sim/tools/setup.sh` to install the pinned dependencies, then launch the simulator.

To import later edits to the original MATLAB inputs, run `python3 flight_sim/tools/import_config.py`. To import GoPro Drone thrust measurements, see the [flight simulator README](flight_sim/README.md#measured-thrust-data).
