
import hloc.extract_features

from pathlib import Path 
import itertools
import yaml
import os
import collections.abc

def main(args=None):

    with open(args.scenario_descriptions_path, "r") as f:
        parameters = yaml.safe_load(f)

    if args.scenario_name is None:
        for experiment_name, experiment in parameters.items():
            # Parse the kvalues to create the illumination levels
            _get_illumination_from_kvalues( experiment )
            create_scenarios_from_template( experiment_name, experiment, args.clear_existing)
    else:
        # Parse the kvalues to create the illumination levels
        _get_illumination_from_kvalues( parameters[args.scenario_name] )
        create_scenarios_from_template( args.scenario_name, parameters[args.scenario_name], args.clear_existing)


def create_scenarios_from_template( experiment_name, scenario_parameters, clear_existing):

    if clear_existing:
        for f in Path(scenario_parameters['scenario_save_dir']).glob('*'):
            f.unlink()

    if not Path(scenario_parameters['scenario_save_dir']).is_dir():
        os.mkdir(scenario_parameters['scenario_save_dir'])

    param_combinations = _get_parameter_permutations(scenario_parameters)
    template_file = Path(scenario_parameters['template_path']).read_text()

    for i, combination in enumerate(param_combinations):
        # Fill the template with the permuted parameters
        filled_template = template_file
        for param_name, param_value in combination.items():
            filled_template = filled_template.replace(param_name+'_template', str(param_value))

        scenario_filename = "combination_{:03d}.xosc".format( i )
        scenario_save_path = Path(scenario_parameters['scenario_save_dir']) / scenario_filename

        filled_template = filled_template.replace("scenario_filepath_template", str(scenario_save_path))
        scenario_save_path.write_text(filled_template)

    print('Created {} scenario files for experiment {}!'.format(i+1, experiment_name))

    parameter_file_save_path = (Path(scenario_parameters['scenario_save_dir']) / 'parameters.yml')
    with open(parameter_file_save_path, 'w') as f:
        yaml.dump(scenario_parameters, f, indent=2)

def _get_illumination_from_kvalues(scenario_params):

    sun_group = []
    for k in scenario_params['sun_group']['k_values']:
        intensity = scenario_params['sun_group']['sun_intensity_startvalue'] * (scenario_params['sun_group']['intensity_decrease_multiplier']**k)
        elevation = scenario_params['sun_group']['sun_elevation_startvalue'] * (scenario_params['sun_group']['elevation_decrease_multiplier']**k)
        sun_group.append( {"sun_intensity": intensity, "sun_elevation": elevation} )
    scenario_params['sun_group'] = sun_group
        
def _get_parameter_permutations(scenario_parameters):

    keys, values = zip(*scenario_parameters.items())

    def check_iterable(value):
        if isinstance(value, collections.abc.Iterable):
            if isinstance(value, str) | isinstance(value, dict):
                return [value]
            else:
                return value
        else:
            return [value]

    # wrap non-iterable values in list to enable permutation using itertools
    iterable_values = list(map( check_iterable, values))

    permutations = [dict(zip(keys, v)) for v in itertools.product(*iterable_values)]

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
        if "image_gallery_path" in param_combination:
            if param_combination["image_gallery_path"] is not None:
                paths = _get_gallery_paths(param_combination["image_gallery_path"],
                                            param_combination["global_extractor_name"],
                                            param_combination["local_extractor_name"],
                                            param_combination["local_matcher_name"])

                global_feature_db_path, local_feature_db_path, sfm_dir_path = paths
                param_combination["gallery_global_descriptor_path"] = global_feature_db_path
                param_combination["gallery_local_descriptor_path"] = local_feature_db_path 
                param_combination["gallery_sfm_path"] = sfm_dir_path

    return permutations

def _get_gallery_paths(image_gallery_path,
                       global_extractor_name,
                       local_extractor_name,
                       local_matcher_name):
    '''
    Get paths of gallery descriptors based on extractor/matcher names
    '''

    extractor_confs = hloc.extract_features.confs.copy()

    combination_identifier = global_extractor_name + '+' + local_extractor_name + '+' + local_matcher_name
    base_path = Path(image_gallery_path) / "outputs" / combination_identifier

    local_feature_db_path = base_path / (extractor_confs[local_extractor_name]['output']+'.h5')
    global_feature_db_path = base_path / (extractor_confs[global_extractor_name]['output']+'.h5')
    sfm_dir_path = base_path / ('sfm_' + combination_identifier)

    return str(global_feature_db_path), str(local_feature_db_path), str(sfm_dir_path)


def parse_arguments():
    import argparse
    from distutils.util import strtobool
    parser = argparse.ArgumentParser(description='CLI test for hloc feature extraction and matching')

    parser.add_argument('--scenario_descriptions_path', type=str, help='Path to yaml file containing scenario parameters',
                        default = '/scenarios/experiment_descriptions.yml')

    parser.add_argument('--scenario_name', type=str, help='(Optional) Name of the entry in file scenario_descriptions_path to use.', default=None)
    parser.add_argument('--clear_existing', type=lambda x: bool(strtobool(x)), default=True)

    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = parse_arguments()
    main(args)