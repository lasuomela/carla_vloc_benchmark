def gallery_capture_parameters( catalog_path ):

    controller_entry_name = ["GalleryCaptureAgent"]

    town_names = ["Town01"]
    route_group = [{"route_entry_name_template": "Map01_testroute_maneuver", "goal_location_x_template": 180.0, "goal_location_y_template": 133.0}]
    goal_location_tolerance = [1]
    target_speed = [8.0]

    # Gallery capture parameters
    image_save_path = ["/image-gallery"]
    # Distance between gallery images in meters
    image_density = [2.0] 

    # Environment parameters
    sun_intensity_startvalue = 1.0
    sun_elevation_startvalue = 1.31
    intensity_decrease_multiplier = 1/2
    elevation_decrease_multiplier = 1/2
    k_values = [0]

    sun_group = []
    for k in k_values:
        intensity = sun_intensity_startvalue * (intensity_decrease_multiplier**k)
        elevation = sun_elevation_startvalue * (elevation_decrease_multiplier**k)
        sun_group.append( {"sun_intensity_template": intensity, "sun_elevation_template": elevation} )

    precipitation_group = [{"precipitation_type_template": "dry", "precipitation_intensity_template": 0.0}]
    visual_range = [ 100000.0]
    cloud_state = ['free']

    scenario_parameters = {
    "town_name_template": town_names,
    "image_save_path_template": image_save_path,
    "image_density_template": image_density,
    "sun_group": sun_group,
    "precipitation_group": precipitation_group,
    "visual_range_template": visual_range,
    "cloud_state_template": cloud_state,
    "route_group": route_group,
    "target_speed_template": target_speed,
    "controller_entry_name_template": controller_entry_name,    
    "goal_location_tolerance_template": goal_location_tolerance,
    "catalog_path_template": catalog_path,
    }
    return scenario_parameters

def test_experiment_parameters(logger_script_path, catalog_path):

    log_file_path = ["/results/test_run.json"]
    log_results = [True]
    controller_entry_name = ["VisualNavigatingAgent"] 

    town_names = ["Town01"]
    route_group = [{"route_entry_name_template": "Map01_testroute_maneuver", "goal_location_x_template": 180.0, "goal_location_y_template": 133.0}]
    goal_location_tolerance = [10]
    target_speed = [8.0]

    # Visual localization specific parameters
    if town_names[0] == "Town01":
        image_gallery_path = ["/image-gallery/2021-10-06_16:43:21/"]
    elif town_names[0] == "Town10HD":
        image_gallery_path = ["/image-gallery/2021-11-15_15:11:29/"] 

    localization_frequence = [2.0]
    top_k_matches = [15]
    global_extractor_name = ["netvlad"]
    local_extractor_group = [{"local_extractor_name_template": "superpoint_aachen", "local_matcher_name_template": "superglue"}]

    # Environment parameters
    sun_intensity_startvalue = 1.0
    sun_elevation_startvalue = 1.31
    intensity_decrease_multiplier = 1/2
    elevation_decrease_multiplier = 1/2
    k_values = [0]

    sun_group = []
    for k in k_values:
        intensity = sun_intensity_startvalue * (intensity_decrease_multiplier**k)
        elevation = sun_elevation_startvalue * (elevation_decrease_multiplier**k)
        sun_group.append( {"sun_intensity_template": intensity, "sun_elevation_template": elevation} )

    precipitation_group = [{"precipitation_type_template": "dry", "precipitation_intensity_template": 0.0}]
    visual_range = [ 100000.0]
    cloud_state = ['free']

    if controller_entry_name[0] == "VisualNavigatingAgent":
        planner_stop_script_path = ["../../opt/ad-runner/src/carla_visual_navigation/utils/run-planner-stop.sh"]
    else:
        planner_stop_script_path = ["../../opt/ad-runner/src/carla_visual_navigation/utils/dummy-script.sh"]

    scenario_parameters = {
    "town_name_template": town_names,
    "global_extractor_name_template": global_extractor_name,
    "local_extractor_group": local_extractor_group,
    "image_gallery_path_template": image_gallery_path,
    "localization_frequence_template": localization_frequence,
    "top_k_matches_template": top_k_matches,
    "sun_group": sun_group,
    "precipitation_group": precipitation_group,
    "visual_range_template": visual_range,
    "cloud_state_template": cloud_state,
    "route_group": route_group,
    "target_speed_template": target_speed,
    "log_file_path_template": log_file_path,
    "log_results_template": log_results,
    "controller_entry_name_template": controller_entry_name,    
    "goal_location_tolerance_template": goal_location_tolerance,
    "logger_script_path_template": logger_script_path,
    "catalog_path_template": catalog_path,
    "planner_stop_script_path_template": planner_stop_script_path,
    }
    return scenario_parameters

def test_experiment_parameters_carla_autopilot(logger_script_path, catalog_path):

    log_file_path = [None]
    log_results = [False]
    controller_entry_name = ["CarlaAutopilotAgent"]

    town_names = ["Town01"]
    route_group = [{"route_entry_name_template": "Map01_testroute_maneuver", "goal_location_x_template": 180.0, "goal_location_y_template": 133.0}]
    goal_location_tolerance = [10]
    target_speed = [8.0]

    # Visual localization specific parameters
    image_gallery_path = [None]
    localization_frequence = [None]
    top_k_matches = [None]
    global_extractor_name = [None]
    local_extractor_group = [{"local_extractor_name_template": None, "local_matcher_name_template": None}]

    # Environment parameters
    sun_intensity_startvalue = 1.0
    sun_elevation_startvalue = 1.31
    intensity_decrease_multiplier = 1/2
    elevation_decrease_multiplier = 1/2
    k_values = [0]

    sun_group = []
    for k in k_values:
        intensity = sun_intensity_startvalue * (intensity_decrease_multiplier**k)
        elevation = sun_elevation_startvalue * (elevation_decrease_multiplier**k)
        sun_group.append( {"sun_intensity_template": intensity, "sun_elevation_template": elevation} )

    precipitation_group = [{"precipitation_type_template": "dry", "precipitation_intensity_template": 0.0}]
    visual_range = [ 100000.0]
    cloud_state = ['free']

    if controller_entry_name[0] == "VisualNavigatingAgent":
        planner_stop_script_path = ["../../opt/ad-runner/src/carla_visual_navigation/utils/run-planner-stop.sh"]
    else:
        planner_stop_script_path = ["../../opt/ad-runner/src/carla_visual_navigation/utils/dummy-script.sh"]

    scenario_parameters = {
    "town_name_template": town_names,
    "global_extractor_name_template": global_extractor_name,
    "local_extractor_group": local_extractor_group,
    "image_gallery_path_template": image_gallery_path,
    "localization_frequence_template": localization_frequence,
    "top_k_matches_template": top_k_matches,
    "sun_group": sun_group,
    "precipitation_group": precipitation_group,
    "visual_range_template": visual_range,
    "cloud_state_template": cloud_state,
    "route_group": route_group,
    "target_speed_template": target_speed,
    "log_file_path_template": log_file_path,
    "log_results_template": log_results,
    "controller_entry_name_template": controller_entry_name,    
    "goal_location_tolerance_template": goal_location_tolerance,
    "logger_script_path_template": logger_script_path,
    "catalog_path_template": catalog_path,
    "planner_stop_script_path_template": planner_stop_script_path,
    }
    return scenario_parameters


def illumination_experiment_parameters(logger_script_path, catalog_path):

    log_file_path = ["/results/town01_illumination_run.json"]
    log_results = ["true"]
    controller_entry_name = ["VisualNavigatingAgent"]

    town_names = ["Town01"]
    route_group = [{"route_entry_name_template": "Map01_route01_maneuver", "goal_location_x_template": 88.5, "goal_location_y_template": 112.0}]
    goal_location_tolerance = [10]
    target_speed = [4.0]

    if town_names[0] == "Town01":
        image_gallery_path = ["/image-gallery/2021-10-06_16:43:21/"]
    elif town_names[0] == "Town10HD":
        image_gallery_path = ["/image-gallery/2021-11-15_15:11:29/"] 

    localization_frequence = [2.0]
    top_k_matches = [15]
    global_extractor_name = ["netvlad", "dir"]
    local_extractor_group = [{"local_extractor_name_template": "sift", "local_matcher_name_template": "NN-ratio"},
                             {"local_extractor_name_template": "superpoint_aachen", "local_matcher_name_template": "superglue"},
                             {"local_extractor_name_template": "d2net-ss", "local_matcher_name_template": "NN-ratio"},
                             {"local_extractor_name_template": "r2d2", "local_matcher_name_template": "NN-ratio" }]

    sun_intensity_startvalue = 1.0
    sun_elevation_startvalue = 1.31
    intensity_decrease_multiplier = 1/2
    elevation_decrease_multiplier = 1/2
    k_values = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

    sun_group = []
    for k in k_values:
        intensity = sun_intensity_startvalue * (intensity_decrease_multiplier**k)
        elevation = sun_elevation_startvalue * (elevation_decrease_multiplier**k)
        sun_group.append( {"sun_intensity_template": intensity, "sun_elevation_template": elevation} )

    precipitation_group = [{"precipitation_type_template": "dry", "precipitation_intensity_template": 0.0}]
    visual_range = [ 100000.0]
    cloud_state = ['free']

    if controller_entry_name[0] == "VisualNavigatingAgent":
        planner_stop_script_path = ["../../opt/ad-runner/src/carla_visual_navigation/utils/run-planner-stop.sh"]
    else:
        planner_stop_script_path = ["../../opt/ad-runner/src/carla_visual_navigation/utils/dummy-script.sh"]

    scenario_parameters = {
    "town_name_template": town_names,
    "global_extractor_name_template": global_extractor_name,
    "local_extractor_group": local_extractor_group,
    "image_gallery_path_template": image_gallery_path,
    "localization_frequence_template": localization_frequence,
    "top_k_matches_template": top_k_matches,
    "sun_group": sun_group,
    "precipitation_group": precipitation_group,
    "visual_range_template": visual_range,
    "cloud_state_template": cloud_state,
    "route_group": route_group,
    "target_speed_template": target_speed,
    "log_file_path_template": log_file_path,
    "log_results_template": log_results,
    "controller_entry_name_template": controller_entry_name,    
    "goal_location_tolerance_template": goal_location_tolerance,
    "logger_script_path_template": logger_script_path,
    "catalog_path_template": catalog_path,
    "planner_stop_script_path_template": planner_stop_script_path,
    }
    return scenario_parameters

def weather_experiment_parameters(logger_script_path, catalog_path):

    log_file_path = ["/results/town10_rainy_run.json"]
    log_results = [True]
    controller_entry_name = ["VisualNavigatingAgent"]

    town_names = ["Town10HD"]
    route_group = [{"route_entry_name_template": "Map10HD_route01_maneuver", "goal_location_x_template": 79.6, "goal_location_y_template": 69.45}]
    goal_location_tolerance = [10]
    target_speed = [4.0]

    if town_names[0] == "Town01":
        image_gallery_path = ["/image-gallery/2021-10-06_16:43:21/"]
    elif town_names[0] == "Town10HD":
        image_gallery_path = ["/image-gallery/2021-11-15_15:11:29/"] 

    localization_frequence = [2.0]
    top_k_matches = [15]
    global_extractor_name = ["netvlad"]
    local_extractor_group = [{"local_extractor_name_template": "superpoint_aachen", "local_matcher_name_template": "superglue"}]

    sun_intensity_startvalue = 1.0
    sun_elevation_startvalue = 1.31
    intensity_decrease_multiplier = 1/2
    elevation_decrease_multiplier = 1/2
    k_values = [6]

    sun_group = []
    for k in k_values:
        intensity = sun_intensity_startvalue * (intensity_decrease_multiplier**k)
        elevation = sun_elevation_startvalue * (elevation_decrease_multiplier**k)
        sun_group.append( {"sun_intensity_template": str(intensity), "sun_elevation_template": str(elevation)} )

    precipitation_group = [{"precipitation_type_template": "rain", "precipitation_intensity_template": 1.0}]
    visual_range = [ 100.0, 10.0, 1.0, 0.1]

    cloud_state = ['rainy']

    if controller_entry_name[0] == "VisualNavigatingAgent":
        planner_stop_script_path = ["../../opt/ad-runner/src/carla_visual_navigation/utils/run-planner-stop.sh"]
    else:
        planner_stop_script_path = ["../../opt/ad-runner/src/carla_visual_navigation/utils/dummy-script.sh"]

    scenario_parameters = {
    "town_name_template": town_names,
    "global_extractor_name_template": global_extractor_name,
    "local_extractor_group": local_extractor_group,
    "image_gallery_path_template": image_gallery_path,
    "localization_frequence_template": localization_frequence,
    "top_k_matches_template": top_k_matches,
    "sun_group": sun_group,
    "precipitation_group": precipitation_group,
    "visual_range_template": visual_range,
    "cloud_state_template": cloud_state,
    "route_group": route_group,
    "target_speed_template": target_speed,
    "log_file_path_template": log_file_path,
    "log_results_template": log_results,
    "controller_entry_name_template": controller_entry_name,    
    "goal_location_tolerance_template": goal_location_tolerance,
    "logger_script_path_template": logger_script_path,
    "catalog_path_template": catalog_path,
    "planner_stop_script_path_template": planner_stop_script_path,
    }
    return scenario_parameters

def streetlight_experiment_parameters(logger_script_path, catalog_path):

    log_file_path = ["/results/town01_streetlight_run.json"]
    log_results = [True]
    controller_entry_name = ["VisualNavigatingAgent"]

    town_names = ["Town01"]
    route_group = [{"route_entry_name_template": "Map01_route01_maneuver", "goal_location_x_template": 88.5, "goal_location_y_template": 112.0}]
    goal_location_tolerance = [10]
    target_speed = [4.0]

    if town_names[0] == "Town01":
        image_gallery_path = ["/image-gallery/2021-10-06_16:43:21/"]
    elif town_names[0] == "Town10HD":
        image_gallery_path = ["/image-gallery/2021-11-15_15:11:29/"] 

    localization_frequence = [2.0]
    top_k_matches = [15]
    global_extractor_name = ["netvlad"]
    local_extractor_group = [{"local_extractor_name_template": "superpoint_aachen", "local_matcher_name_template": "superglue"}]

    sun_intensity_startvalue = 0.0
    sun_elevation_startvalue = -1.31
    intensity_decrease_multiplier = 1/2
    elevation_decrease_multiplier = 1/2
    k_values = [0]

    sun_group = []
    for k in k_values:
        intensity = sun_intensity_startvalue * (intensity_decrease_multiplier**k)
        elevation = sun_elevation_startvalue * (elevation_decrease_multiplier**k)
        sun_group.append( {"sun_intensity_template": intensity, "sun_elevation_template": elevation} )

    precipitation_group = [{"precipitation_type_template": "dry", "precipitation_intensity_template": 0.0}]
    visual_range = [ 100000.0]
    cloud_state = ['free']

    if controller_entry_name[0] == "VisualNavigatingAgent":
        planner_stop_script_path = ["../../opt/ad-runner/src/carla_visual_navigation/utils/run-planner-stop.sh"]
    else:
        planner_stop_script_path = ["../../opt/ad-runner/src/carla_visual_navigation/utils/dummy-script.sh"]

    scenario_parameters = {
    "town_name_template": town_names,
    "global_extractor_name_template": global_extractor_name,
    "local_extractor_group": local_extractor_group,
    "image_gallery_path_template": image_gallery_path,
    "localization_frequence_template": localization_frequence,
    "top_k_matches_template": top_k_matches,
    "sun_group": sun_group,
    "precipitation_group": precipitation_group,
    "visual_range_template": visual_range,
    "cloud_state_template": cloud_state,
    "route_group": route_group,
    "target_speed_template": target_speed,
    "log_file_path_template": log_file_path,
    "log_results_template": log_results,
    "controller_entry_name_template": controller_entry_name,    
    "goal_location_tolerance_template": goal_location_tolerance,
    "logger_script_path_template": logger_script_path,
    "catalog_path_template": catalog_path,
    "planner_stop_script_path_template": planner_stop_script_path,
    }
    return scenario_parameters
