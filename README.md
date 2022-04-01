
<p align="center">
  <img src="doc/overview.png" width="100%"/></a>
  <br />
</p>

# Carla Visual localization benchmark

Benchmark visual localization methods for robot navigation.

[Paper](https://arxiv.org/abs/2203.13048) |
[Video](https://youtu.be/OGjrOt_N1xM)

>A Simulation Benchmark for Vision-based Autonomous Navigation   
>Lauri Suomela, Atakan Dag, Harry Edelman, Joni-Kristian Kämäräinen  
>arXiv

## Installation

*The code is tested on Ubuntu 20.04 with one Nvidia RTX3090. As everything runs inside docker containers, supporting platforms other than linux should only require modifying the build and run scripts in the docker folder.*

Pull the repository:

```sh
git clone https://github.com/lasuomela/carla_vloc_benchmark/
git submodule update --init --recursive
```

We strongly recommend running the benchmark inside the provided docker images. Build the images:

```sh
cd docker
./build-carla.sh
./build-ros-bridge-scenario.sh
```

Next, validate that the environment is correctly set up. Launch the Carla simulator:

```sh
# With GUI
./run-carla.sh
```

In another terminal window, start the autonomous agent / scenario runner container:

```sh
cd docker
./run-ros-bridge-scenario.sh
```

Now, inside the container terminal:

```sh
cd /opt/visual_robot_localization/src/visual_robot_localization/utils

# Run SfM with the example images included with the visual_robot_localization package.
./do_SfM.sh

# Visualize the resulting model
./visualize_colmap.sh

# Test that the package can localize agains the model
launch_test /opt/visual_robot_localization/src/visual_robot_localization/test/visual_pose_estimator_test.launch.py
```

If all functions correctly, you are good to go.

## Reproduce the paper experiments

### 1. Launch the environment
In one terminal window, launch the Carla simulator:

```sh
cd docker
# Headless
./run-bash-carla.sh
```

In another terminal window, launch a container which contains the autonomous agent and scenario execution logic:

```sh
cd docker
./run-ros-bridge-scenario.sh
```

The run command mounts the `/carla_visual_navigation`, `/image-gallery`, `/scenarios` and `/results` folders into the container, so the changes to these folders are reflected both inside the container and on the host system.

### 2. Capture the gallery datasets

When running for the first time, you need to capture the images from the test route for 3D reconstruction.

Inside the container terminal from last step:

```sh
cd /opt/carla_visual_navigation/src/carla_visual_navigation/scripts

# Generate the scenario files describing gallery image capture and experiments
# See /scenarios/experiment_descriptions.yml for list of experiments
python template_scenario_generator.py

# Launch terminal multiplexer
tmux

# Create a second tmux window into the container using 'shift-ctrl-b' + '%'
# Tmux window 1: start the scenario runner with rviz GUI
ros2 launch carla_visual_navigation rviz_scenario_runner.launch.py town:='Town01'

# Tmux window 2: start gallery capture execution
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/gallery_capture_town01_route01' repetitions:=1
```

If everything went okay, you should now have a folder `/image-gallery/town01_route1` with images captured along the route. Repeat for Town10:

```sh
# Tmux window 1
ros2 launch carla_visual_navigation rviz_scenario_runner.launch.py town:='Town10HD'

# Tmux window 2
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/gallery_capture_town10_route01' repetitions:=1
```

### 3. Run 3D reconstruction for the gallery images

Next, we triangulate sparse 3D models from the gallery images and their camera poses. The models are saved in the respective image capture folders in `/image-gallery/` 

```sh
cd /opt/carla_visual_navigation/src/carla_visual_navigation/scripts

# Check the visual localization methods used in the experiments and run 3D reconstruction with each of them
python create_gallery_reconstructions.py --experiment_path '/scenarios/illumination_experiment_town01'

# Repeat for Town10
python create_gallery_reconstructions.py --experiment_path '/scenarios/illumination_experiment_town10'
```

You can visually examine the reconstructions with

```sh
./visualize_colmap.sh --image_folder '/image-gallery/town01_route1' --localization_combination_name 'netvlad+superpoint_aachen+superglue'
```

### 4. Run the experiments

To replicate the experiment results:


```sh
# Launch terminal multiplexer
tmux

# Create a second tmux window into the container using 'shift-ctrl-b' + '%'
# In the first window, start the scenario runner headless (the rviz GUI slows the simulation down)
ros2 launch carla_visual_navigation cli_scenario_runner.launch.py town:='Town01'

# In the second window, start scenario execution for Town01, with 5 repetitions of each scenario file
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/illumination_experiment_town01_route01' repetitions:=5

# Once all the scenarios have been finished, run the scenarios with autopilot to measure visual localization recall
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/illumination_experiment_town01_route01_autopilot' repetitions:=1

# Next, measure navigation performance with wheel odometry only
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/illumination_experiment_town01_route01_odometry_only' repetitions:=5
```

Completing the experiments can take a long time. Once the experiments have been completed, repeat for the Town10 envrionment. The results are saved to `/results/`. To produce aggregated metrics, you can run

```sh
cd /opt/carla_visual_navigation/src/carla_visual_navigation/scripts
python analyze_illumination_scenario_results.py
```

## Adding new visual localization methods

The visual localization methods are integrated through the awesome [hloc toolbox](https://github.com/cvg/Hierarchical-Localization), for which we provide a ROS2 wrapper in the [visual_robot_localization](https://github.com/lasuomela/visual_robot_localization) package. See hloc documentation on how to contribute new visual localization methods. After a method has been added to hloc, it can be used in the experiments by specifying the method in the experiment parameters (`/scenarios/experiment_descriptions.yml`)


## Defining your own experiments

Documentation TBD. See `/scenarios/experiment_descriptions.yml` and the scenario template `carla_vloc_benchmark/carla_visual_navigation/config/VisualNavigatorTemplate.xosc` for some options.

