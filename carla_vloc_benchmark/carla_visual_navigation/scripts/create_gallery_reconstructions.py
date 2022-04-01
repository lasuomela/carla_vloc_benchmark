import sys
from pathlib import Path
import yaml

dir_path = Path(__file__).parent / '../../third_party/deep-image-retrieval'
sys.path.append('/opt/visual_robot_localization/src/visual_robot_localization/scripts/')

import SfM_from_camera_poses


def main(args=None):

    with open(Path(args.experiment_path) / 'parameters.yml', "r") as f:
        parameters = yaml.safe_load(f)

    for global_extractor in parameters['global_extractor_name']:
        for local_extractor_and_matcher in parameters['local_extractor_group']:

            print('\nRunning point triangulation for combination {} + {} + {}\n'.format(global_extractor,
                                                                                    local_extractor_and_matcher['local_extractor_name'],
                                                                                    local_extractor_and_matcher['local_matcher_name']))

            SfM_from_camera_poses.main(parameters['image_gallery_path'],
                                    global_extractor,
                                    local_extractor_and_matcher['local_extractor_name'],
                                    local_extractor_and_matcher['local_matcher_name'],
                                    args.n_matches,
                                    args.im_size_x,
                                    args.im_size_y,
                                    args.im_fov)


def parse_arguments():
    import argparse
    parser = argparse.ArgumentParser(description='Run 3D model reconstruction with methods used in an experiment')

    parser.add_argument('--experiment_path', type=str, help='Path to directory which contains the experiment scenario files')

    parser.add_argument('--n_matches', type=int, help='number of best matching images to use for reconstruction', default=25)
    parser.add_argument('--im_size_x', type=int, help='Horizontal size in pixels of the gallery images', default=800)
    parser.add_argument('--im_size_y', type=int, help='Vertical size in pixels of the gallery images', default=600)
    parser.add_argument('--im_fov', type=int, help='Field of view width of the camera', default=90)

    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = parse_arguments()
    main(args)