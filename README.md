
# frankie_planner

Minimal example of frankie planning pipeline using moveit

## Explanation

This repo consider the required moveit2 library including the ros dependencies, inside the src folder we have the different ros packages including:
```
- frankie_description -> Robot description
- frankie_moveit_config -> Planning configuration, considering the non-holonomic constraints of the robot
- frankie_planner -> launch files and scene descriptions
- stretch_kinematics_plugin -> Modified version of the stretch kinematics plugin to sample ik solutions
```

### Minor details
The modifications for the ik plugin consider in a nutshell that we always start with a random seed to avoid re using a position of the base previously computed

## Usage
Just run the following:

Only first time
```
pixi run build
```

Then on one terminal to launch the visualization, controllers and moveit configs
```
pixi run rviz 
```

Finally to run the planning pipeling using moveit_py
```
pixi run python scripts/load_rmmi.py
```
