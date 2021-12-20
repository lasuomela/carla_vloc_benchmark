
import hloc.extract_features
from scenario_parameters import weather_experiment_parameters, test_experiment_parameters, test_experiment_parameters_carla_autopilot, gallery_capture_parameters

from pathlib import Path 
from copy import deepcopy
import itertools
import json
import os

def main(args=None):

    scenario_save_dir = '/scenarios/template_generated_test'

    template_path = '/opt/carla_vloc_benchmark/src/carla_visual_navigation/config/VisualNavigatorTemplate.xosc'
    #template_path = '/opt/carla_vloc_benchmark/src/place_reg_ad/config/GalleryCaptureTemplate.xosc'


    # Paths relative to location of the generated scenario file
    logger_script_path = ["../../opt/carla_vloc_benchmark/src/carla_visual_navigation/utils/run-scenario-log.sh"]          
    catalog_path = ["../../opt/carla_vloc_benchmark/src/carla_visual_navigation/config/catalogs"]

    scenario_parameters = test_experiment_parameters(logger_script_path, catalog_path)

    #scenario_parameters =  gallery_capture_parameters(catalog_path)

    clear_existing = True 
    create_scenarios_from_template(template_path, scenario_save_dir, scenario_parameters, clear_existing)

def create_scenarios_from_template(template_path, scenario_save_dir, scenario_parameters, clear_existing):

    if clear_existing:
        for f in Path(scenario_save_dir).glob('*'):
            f.unlink()

    if not Path(scenario_save_dir).is_dir():
        os.mkdir(scenario_save_dir)

    param_combinations = _get_parameter_permutations(scenario_parameters)
    template_file = Path(template_path).read_text()

    for i, combination in enumerate(param_combinations):
        filled_template = template_file
        for param_name, param_value in combination.items():
            filled_template = filled_template.replace(param_name, str(param_value))

        scenario_filename = "combination_{:03d}.xosc".format( i )
        scenario_save_path = Path(scenario_save_dir) / scenario_filename

        filled_template = filled_template.replace("scenario_filepath_template", str(scenario_save_path))
        scenario_save_path.write_text(filled_template)

    print('Created {} scenario files!'.format(i+1))

    parameter_file_save_path = (Path(scenario_save_dir) / 'parameters.json')
    with open(parameter_file_save_path, 'w') as f:
        json.dump(scenario_parameters, f, indent=2)
        
def _get_parameter_permutations(scenario_parameters):

    keys, values = zip(*scenario_parameters.items())
    permutations = [dict(zip(keys, v)) for v in itertools.product(*values)]

    for param_combination in permutations:
        # Flatten the nested dicts (parameter_groups)
        for param_name, param_value in list(param_combination.items()):
            if isinstance(param_value, dict):
                # Add the parameter values into the outer dict
                for key, value in param_value.items():
                    param_combination[key] = value
                # Remove the group
                del param_combination[param_name]

        # Retrieve database paths if applicable
        if "image_gallery_path_template" in param_combination:
            if param_combination["image_gallery_path_template"] is not None:
                paths = _get_gallery_paths(param_combination["image_gallery_path_template"],
                                            param_combination["global_extractor_name_template"],
                                            param_combination["local_extractor_name_template"],
                                            param_combination["local_matcher_name_template"])

                global_feature_db_path, local_feature_db_path, sfm_dir_path = paths
                param_combination["gallery_global_descriptor_path_template"] = global_feature_db_path
                param_combination["gallery_local_descriptor_path_template"] = local_feature_db_path 
                param_combination["gallery_sfm_path_template"] = sfm_dir_path

    return permutations

def _get_gallery_paths(image_gallery_path,
                       global_extractor_name,
                       local_extractor_name,
                       local_matcher_name):
    '''
    Get paths of gallery descriptors based on extractor/matcher names
    '''

    extractor_confs = hloc.extract_features.confs.copy()
    # Manually add r2d2 extractor configuration since the hloc pr hasn't been merged yet
    # https://github.com/cvg/Hierarchical-Localization/pull/85/commits
    extractor_confs['r2d2'] = {
        'output': 'feats-r2d2-n5000-r1024',
        'model':{
            'name': 'r2d2',
            'max_keypoints': 5000,
        },
        'preprocessing': {
            'grayscale': False,
            'resize_max': 1024,
        },
    }

    combination_identifier = global_extractor_name + '+' + local_extractor_name + '+' + local_matcher_name
    base_path = Path(image_gallery_path) / "outputs" / combination_identifier

    local_feature_db_path = base_path / (extractor_confs[local_extractor_name]['output']+'.h5')
    global_feature_db_path = base_path / (extractor_confs[global_extractor_name]['output']+'.h5')
    sfm_dir_path = base_path / ('sfm_' + combination_identifier)

    return str(global_feature_db_path), str(local_feature_db_path), str(sfm_dir_path)

if __name__ == '__main__':
    main()
