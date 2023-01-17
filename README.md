
<p align="center">
  <img src="doc/overview.png" width="100%"/></a>
  <br />
</p>

# Carla Visual localization benchmark

[Paper](https://arxiv.org/abs/2203.13048) |
[Webpage](https://lasuomela.github.io/carla_vloc_benchmark/)

This is the official implementation of the paper "Benchmarking Visual Localization for Autonomous Navigation".

The benchmark enables easy experimentation with different visual localization methods as part of a navigation stack. The platform enables investigating how various factors such as illumination, viewpoint, and weather changes affect visual localization and subsequent navigation performance. The benchmark is based on the Carla autonomous driving simulator and our ROS2 port of the Hloc visual localization toolbox.

## Citing

If you find the benchmark useful in your research, please cite our work as:
```
@InProceedings{Suomela_2023_WACV,
    author    = {Suomela, Lauri and Kalliola, Jussi and Dag, Atakan and Edelman, Harry and Kämäräinen, Joni-Kristian},
    title     = {Benchmarking Visual Localization for Autonomous Navigation},
    booktitle = {Proceedings of the IEEE/CVF Winter Conference on Applications of Computer Vision (WACV)},
    month     = {January},
    year      = {2023},
    pages     = {2945-2955}
}
```

## System requirements

- [Docker](https://www.docker.com/)
- [Nvidia-docker](https://github.com/NVIDIA/nvidia-docker)
- Nvidia GPU with minimum of 12GB memory. *Recommended Nvidia RTX3090*
- 70GB disk space for Docker images

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

---

<details><summary><b><font size="+2">Reproduce the paper experiments</font></b></summary>

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
cd /opt/carla_vloc_benchmark/src/carla_visual_navigation/scripts

# Generate the scenario files describing gallery image capture and experiments
# See /scenarios/experiment_descriptions.yml for list of experiments
python template_scenario_creator.py

# Launch terminal multiplexer
tmux

# Create a second tmux window into the container using 'shift-ctrl-b' + '%'
# Tmux window 1: start the scenario runner with rviz GUI
ros2 launch carla_visual_navigation rviz_scenario_runner.launch.py town:='Town01'

# Tmux window 2: start gallery capture execution
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/gallery_capture_town01_route1' repetitions:=1
```

If everything went okay, you should now have a folder `/image-gallery/town01_route1` 
with images captured along the route. Repeat for Town10:

```sh
# Tmux window 1
ros2 launch carla_visual_navigation rviz_scenario_runner.launch.py town:='Town10HD'

# Tmux window 2
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/town10_route1' repetitions:=1
```

### 3. Run 3D reconstruction for the gallery images

Next, we triangulate sparse 3D models from the gallery images and their camera poses. 
The models are saved in the respective image capture folders in `/image-gallery/`

```sh
cd /opt/carla_vloc_benchmark/src/carla_visual_navigation/scripts

# Check the visual localization methods used in the experiments and run 3D reconstruction with each of them
python create_gallery_reconstructions.py --experiment_path '/scenarios/illumination_experiment_town01'

# Repeat for Town10
python create_gallery_reconstructions.py --experiment_path '/scenarios/illumination_experiment_town10'
```

You can visually examine the reconstructions with

```sh
cd /opt/visual_robot_localization/src/visual_robot_localization/utils

./visualize_colmap.sh --image_folder '/image-gallery/town01_route1' --localization_combination_name 'netvlad+superpoint_aachen+superglue'
```

### 4. Run the illumination experiments

To replicate the experiment results:

```sh
# Launch terminal multiplexer
tmux

# Create a second tmux window into the container using 'shift-ctrl-b' + '%'
# In the first window, start the scenario runner headless (the rviz GUI slows the simulation down)
ros2 launch carla_visual_navigation cli_scenario_runner.launch.py town:='Town01'

# In the second window, start scenario execution for Town01, with 5 repetitions of each scenario file
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/illumination_experiment_town01' repetitions:=5

# Once all the scenarios have been finished, run the scenarios with autopilot to measure visual localization recall
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/illumination_experiment_town01_autopilot' repetitions:=1

# Next, measure navigation performance with wheel odometry only
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/illumination_experiment_town01_odometry_only' repetitions:=5
```

Completing the experiments can take a long time. Once the experiments have been completed, 
repeat for the Town10 envrionment. The results are saved to `/results/`.

### 5. Run the viewpoint change experiments
Viewpoint change -experiments uses the same gallery images and sparse 3D models as in the illumination experiments.
Each viewpoint change (e.g., change in camera angle) has its own experiment description 
in `scenarios/experiment_descriptions.yml` and sensor configuration file in  
`carla_vloc_benchmark/carla_visual_navigation/config/viewpoint_experiment_objects/`.

Run an experiment with the following camera position: `z=4.0, pitch=10.0`
```sh
# Tmux window 1
ros2 launch carla_visual_navigation cli_scenario_runner.launch.py town:='Town01' objects_config:='/opt/carla_vloc_benchmark/src/carla_visual_navigation/config/viewpoint_experiment_objects/zpitch/objects_zpitch1.json'

# In Tmux window 2, start scenario execution for Town01 and 5 repetitions of each scenario file
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/viewpoint_experiment_zpitch1_town01' repetitions:=5

# Once all the scenarios have been finished, run the scenarios with autopilot to measure visual localization recall
# Remember to define the same sensor configuration file as previously

# Tmux window 1
ros2 launch carla_visual_navigation cli_scenario_runner.launch.py town:='Town01' objects_config:='/opt/carla_vloc_benchmark/src/carla_visual_navigation/config/viewpoint_experiment_objects/zpitch/objects_zpitch1.json'

# Tmux window 2
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/viewpoint_experiment_zpitch1_town01_autopilot' repetitions:=1

# If Illumination experiments are performed, then there is no need to measure navigation performance with wheel odometry only. Same results can be used for viewpoint experiments

# Otherwise, measure navigation performance with wheel odometry only
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/illumination_experiment_town01_odometry_only' repetitions:=5
```

Repeat the previous commands for other camera positions _(zpitch2, zpitch3, zpitch4, ...)_ 
and viewpoint changes _(roll, yaw)_ The results are saved to `/results/`.

<details><summary><b>List of the sensor configurations</b></summary>

<table>
<tr style="vertical-align:top;border:none;"><td style="vertical-align:top;border:none;">

| Filename              | Pitch | Z    |
|-----------------------|-------|------|
| objects_zpitch1.json  | 10.0  | 4.0  |
| objects_zpitch2.json  | 22.5  | 6.0  |
| objects_zpitch3.json  | 27.5  | 7.0  |
| objects_zpitch4.json  | 32.5  | 8.0  |
| objects_zpitch5.json  | 35.0  | 9.0  |
| objects_zpitch6.json  | 37.5  | 10.0 |
| objects_zpitch7.json  | 40.0  | 11.0 |
| objects_zpitch8.json  | 40.0  | 12.0 |
| objects_zpitch9.json  | 40.0  | 13.0 |
| objects_zpitch10.json | 40.0  | 15.0 |
| objects_zpitch11.json | 40.0  | 17.0 |
| objects_zpitch12.json | 40.0  | 18.0 |
</td><td style="vertical-align:top;border:none;">

| Filename          | Yaw   |
|-------------------|-------|
| objects_yaw1.json | 90.0  |
| objects_yaw2.json | 67.5  |
| objects_yaw3.json | 45.0  |
| objects_yaw4.json | 22.5  |
| objects_yaw5.json | -22.5 |
| objects_yaw6.json | -45.0 |
| objects_yaw7.json | -67.5 |
| objects_yaw8.json | -90.0 |

</td></tr>
</table>


</details>

---

#### Alternative way to run the viewpoint experiments is to use following command:

```shell
cd /opt/carla_vloc_benchmark/src/carla_visual_navigation/scripts

# Execute all scenarios which starts with "viewpoint_experiment_yaw"
./run_viewpoint_experiments.sh -e viewpoint_experiment_zpitch -a True -t Town01
```
This will open tmux session where scenario executor and scenario runner -commands are executed using
defined parameters. Above command will run all the viewpoint experiments which starts with `viewpoint_experiment_zpitch`
and corresponding autopilot experiments. Script will also restart the experiments from the previous 
run if the execution stops for some reason. 

The script uses following parameters:
- `-e`: **Experiment name** (**Required**, String), can be used to run exact experiments like `viewpoint_experiment_zpitch1` or all the 
experiments belonging to the same category `viewpoint_experiment_zpitch`.
- `-t`: **Town name** (**Required**, String)
- `-a`: **Autopilot** (Optional, Boolean), if corresponding autopilot experiments are executed after. _Default value: False_
- `-o`: **Odometry** (Optional, Boolean), if odometry experiment is executed after. _Default value: False_
- `-r`: **Repetitions** (Optional, Int) how many times each experiment should be executed. _Default value: 5_
- `-n`: **Exact experiment name** (Optional, Boolean), if exact experiment name should be used. _Default value: False_

---

### 6. Run the weather experiments

Run the experiments with weather changes:
```sh
# Tmux window 1
ros2 launch carla_visual_navigation cli_scenario_runner.launch.py town:='Town10HD'

# Tmux window 2
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/weather_experiment_town10' repetitions:=5

# Once all the scenarios have been finished, run the scenarios with autopilot to measure visual localization recall
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/weather_experiment_town10_autopilot' repetitions:=1

# If Illumination experiments are performed, then there is no need to measure navigation performance with wheel odometry only. Same results can be used for viewpoint experiments

# Otherwise, measure navigation performance with wheel odometry only
ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/illumination_experiment_town10_odometry_only' repetitions:=5
```

### 7. Produce aggregate metrics
Produce aggregate metrics and plots from the results.

```sh
cd /opt/carla_visual_navigation/src/carla_visual_navigation/scripts
python analyze_scenario_logs.py
```

</details>

---

<details><summary><b><font size="+2">Adding new visual localization methods</font></b></summary>

The visual localization methods are integrated through the awesome 
[hloc toolbox](https://github.com/cvg/Hierarchical-Localization), 
for which we provide a ROS2 wrapper in the 
[visual_robot_localization](https://github.com/lasuomela/visual_robot_localization) package. 
See hloc documentation on how to contribute new visual localization methods. 
After a method has been added to hloc, it can be used in the experiments by specifying the method 
in the experiment parameters (`/scenarios/experiment_descriptions.yml`)

</details>

---
<details><summary><b><font size="+2">Defining your own experiments</font></b></summary>

### 1. Define experiment descriptions
Experiments are defined in `/scenarios/experiment_descriptions.yml` using YAML. 
Experiment descriptions should contain all the parameters and parameter combinations used in 
scenarios. **Parameter naming should be the same as in the OpenSCENARIO template files.**

Example experiment definition structure:
```sh
 # Experiment name
 experiment_1_name:
   # The idea is to create scenario files that contain all the possible permutations of the parameters specified for an experiment.
   parameter_1_name: parameter_1_value
   parameter_2_name: parameter_2_value
   parameter_3_name: [parameter_3_value_a, parameter_3_value_b, parameter_3_value_c]
   # Sometimes parameters needs to be grouped together, e.g., SuperPoint extractor with Superglue matcher. 
   parameter_4_group:
     - group_param_1_a_name: group_param_1_a_value
       group_param_1_b_name: group_param_1_b_value

     - group_param_2_a_name: group_param_2_a_value
       group_param_2_b_name: group_param_2_b_value

 experiment_2_name:
   etc.
```
See `/scenarios/experiment_descriptions.yml` for more examples and details.


### 2. Define OpenSCENARIO template
ASAM OpenSCENARIO templates are defined in `carla_vloc_benchmark/carla_visual_navigation/config` using XML. 
Experiment parameters defined in `/scenarios/experiment_descriptions.yml` are used to 
populate the values in scenario template.

Example OpenScenario template structure:
```sh
<?xml version="1.0"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="0" date="2021-05-04T00:00:00" description="CARLA:TemplateName" author="author"/>
  <ParameterDeclarations>
    <ParameterDeclaration name="town_name" parameterType="string" value="town_name_template"/>
    <ParameterDeclaration name="ekf_config_path" parameterType="string" value="ekf_config_path_template"/>
    <ParameterDeclaration name="global_extractor_name" parameterType="string" value="global_extractor_name_template"/>
    ...
  </ParameterDeclarations>
  <CatalogLocations>
    <ControllerCatalog>
      <Directory path="catalog_path_template"/>
    </ControllerCatalog>
    ...
  </CatalogLocations>
  <RoadNetwork>
    ...
  </RoadNetwork>
  <Entities>
    ...
  </Entities>
  <Storyboard>
    ...
  </Storyboard>
</OpenSCENARIO>
```

See [ASAM OpenSCENARIO User guide](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html) and [documentation](https://releases.asam.net/OpenSCENARIO/1.0.0/Model-Documentation/index.html) for detailed instructions and supported parameters. 

Example template can be seen in `carla_vloc_benchmark/carla_visual_navigation/config/VisualNavigatorTemplate.xosc`. 

### 3. Define OpenSCENARIO catalog
ASAM OpenSCENARIO catalogs are defined in `carla_vloc_benchmark/carla_visual_navigation/config/catalogs` using XML. Catalogs contain the vehicle, controller, environment and route specifications, which are provided in this repository.  

Example OpenSCENARIO catalog definition structure _(vehicle)_:
```shell
<?xml version="1.0" encoding="UTF-8"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="0" date="2020-03-20T00:00:00" description="CARLA:ControllerCatalog" author="" />
  <Catalog name="VehicleCatalog">
    <Vehicle name="vehicle.tesla.model3" vehicleCategory="car">
        <ParameterDeclarations/>
        <Performance maxSpeed="69.444" maxAcceleration="10.0" maxDeceleration="10.0"/>
        <BoundingBox>
          <Center x="1.5" y="0.0" z="0.9"/>
          <Dimensions width="2.1" length="4.5" height="1.8"/>
        </BoundingBox>
        <Axles>
          <FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8" positionX="3.1" positionZ="0.3"/>
          <RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="0.0" positionZ="0.3"/>
        </Axles>
        <Properties>
          <Property name="type" value="ego_vehicle"/>
          <Property name="color" value="0,0,255"/>
        </Properties>
      </Vehicle>
  </Catalog>
</OpenSCENARIO>
```

See [ASAM OpenSCENARIO User guide](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html) and [documentation](https://releases.asam.net/OpenSCENARIO/1.0.0/Model-Documentation/index.html) for detailed instructions and supported parameters.

### 4. Generate scenario files
Scenario files can be generated if experiments are defined in `scenarios/experiment_descriptions.yml`, 
OpenSCENARIO templates in `carla_vloc_benchmark/carla_visual_navigation/config` 
and vehicle, controller, environment and route specification catalogs in 
`carla_vloc_benchmark/carla_visual_navigation/config/catalogs` using following commands:

```shell
# Launch a container which contains the autonomous agent and scenario execution logic
cd docker
./run-ros-bridge-scenario.sh

cd /opt/carla_vloc_benchmark/src/carla_visual_navigation/scripts

# Generate the scenario files described in /scenarios/experiment_descriptions.yml
python template_scenario_creator.py
```

`template_scneario_creator.py` populates the OpenSCENARIO templates with parameter values 
from experiment descriptions and creates a scenario file for each parameter combination. 
Populated scenario files and parameter file are stored in `scenarios/experiment_name` 
with names `combination_000.xosc, combination_001.xosc, ...` and `parameters.yml`.
</details>

---




