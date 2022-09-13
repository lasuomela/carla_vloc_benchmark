import subprocess
import sys
import argparse
import os
import time
from pathlib import Path
import yaml
import json

import rclpy
from rclpy.node import Node

from carla_ros_scenario_runner_types.msg import CarlaScenario, CarlaScenarioRunnerStatus
from carla_ros_scenario_runner_types.srv import ExecuteScenario

EXPERIMENT=''
TOWN='Town01'
AUTOPILOT=False
ODOMETRY=False
REPETITIONS=1
EXACT_NAME=False
node = None
experiments=None
object_configs=None



def install_and_import(package):
    import importlib
    try:
        importlib.import_module(package)
    except ImportError:
        import pip
        pip.main(['install', package])
    finally:
        globals()[package] = importlib.import_module(package)

def get_object_config_file(experiment_path):
    splitted_experiment = experiment_path.split('_')
    experiment_base = splitted_experiment[0]
    object_configs=[]

    if experiment_base == 'viewpoint':
        method = splitted_experiment[2]
        method_base = ''.join(i for i in method if not i.isdigit())
        objects_config_path = os.path.join('/opt/carla_vloc_benchmark/src/carla_visual_navigation/config/', 'viewpoint_experiment_objects', method_base)

        object_configs = [f for f in os.listdir(objects_config_path) if os.path.isfile(os.path.join(objects_config_path, f))]

    elif experiment_base == 'illumination':
        object_configs = ['/opt/carla_vloc_benchmark/src/carla_visual_navigation/config/objects_noview.json']
    else:
        print('Only viewpoint and illumination are implemented for now.')

    return object_configs

def read_experiments(experiment_path, town, autopilot, odometry, exact_name):
    town=town.lower()
    
    if town == 'town10hd':
        town = 'town10'

    experiment_path=experiment_path.lower()

    experiment_folders=[]
    autopilot_folders=[]
    odometry_folders=[]

    print(experiment_path, town, autopilot, odometry, exact_name)

    if 'gallery' in experiment_path:
        odometry=False
        autopilot=False

    scenarios_path = '/scenarios'
    
    # Run only the experiments which have exactly the same name as given by the user
    if exact_name:
        target_name=experiment_path + '_' + town
        
        for f in os.listdir(scenarios_path):
            folder_path=os.path.join(scenarios_path, f)
            
            if os.path.isdir(folder_path) and f == target_name and 'autopilot' not in f and 'odometry' not in f:
                experiment_folders.append(folder_path)
            if autopilot and os.path.isdir(folder_path) and f == target_name + '_autopilot':
                autopilot_folders.append(folder_path)
            if odometry and os.path.isdir(folder_path) and f == target_name + '_odometry':
                odometry_folders.append(folder_path)
    # Run all the experiments which starts with the given experiment name
    else:
        
        for f in os.listdir(scenarios_path):
            folder_path=os.path.join(scenarios_path, f)
            
            if os.path.isdir(folder_path) and f.startswith(experiment_path) and town in f and 'autopilot' not in f and 'odometry' not in f:
                experiment_folders.append(folder_path)
            if autopilot and os.path.isdir(folder_path) and f.startswith(experiment_path)  and town in f and 'autopilot' in f and 'odometry' not in f:
                autopilot_folders.append(folder_path)
            if odometry and os.path.isdir(folder_path) and f.startswith(experiment_path) and town in f and 'odometry' in f and 'autopilot' not in f:
                odometry_folders.append(folder_path)

    if len(experiment_folders) == 0:
        sys.exit(f'No experiments found with experiment={experiment_path} and town={town}.')

    # If user wants to do odometry but there is no odometry files
    if len(odometry_folders) == 0 and odometry:
        cont = input('didnt find any odometry file. Are you sure you want to continue? [y/n]')
        if cont.lower() not in ['yes', 'y']:
            sys.exit("Exited from the script.")

    # Check that there is the same number of autopilot and experiment files
    #if (len(autopilot_folders) != len(experiment_folders) or len(autopilot_folders) == 0) and autopilot:
    #    print(experiment_folders, autopilot_folders) 
    #    cont = input('There is a mismatch between files or there is no autopilot files. Are you sure you want to continue? [y/n]')
    #    if cont.lower() not in ['yes', 'y']:
    #        sys.exit("Exited from the script.")

    

    object_configs_temp = get_object_config_file(experiment_path=experiment_path)
    object_configs=[]

    def prune_scenarios(scenarios):
        
        remove_idxs = []
        object_configs=[]

        for idx, scenario in enumerate(scenarios):
            scenario_without_autopilot = scenario.replace('_autopilot', '')
            splitted_scenario = scenario_without_autopilot.split('_')
            
            remove_scenario = True
            if 'illumination' in splitted_scenario[0]:
                object_configs.append(f'/opt/carla_vloc_benchmark/src/carla_visual_navigation/config/objects_noview.json')
            elif 'viewpoint' in splitted_scenario[0]:
                obj = splitted_scenario[2]
                for config in object_configs_temp:
                    temp_config = config.replace('.json', '')
                    temp_config = temp_config.replace('objects_', '')
                    if obj == temp_config:
                        config_method = config.split('.')[0]
                        config_method = config_method.split('_')[1]
                        config_method = ''.join(i for i in config_method if not i.isdigit())
                        config_path=f'/opt/carla_vloc_benchmark/src/carla_visual_navigation/config/viewpoint_experiment_objects/{config_method}/{config}'
                        
                        
                        if os.path.isfile(config_path):
                            remove_scenario = False
                            object_configs.append(f'/opt/carla_vloc_benchmark/src/carla_visual_navigation/config/viewpoint_experiment_objects/{config_method}/{config}')
                            break
                        else:
                            raise Exception(f'No {config_method} object config file found. please create a file with correct name and try again.')
                            sys.exit(f'No {config_method} object config file found. please create a file with correct name and try again.')
                    else:
                        remove_scenario = True
                if remove_scenario:
                    remove_idxs.append(idx)
        for idx in sorted(remove_idxs, reverse=True):
            del scenarios[idx]

        return scenarios, object_configs

    experiment_folders, experiment_object_configs = prune_scenarios(experiment_folders)
    autopilot_folders, autopilot_object_configs = prune_scenarios(autopilot_folders)

    object_configs = experiment_object_configs + autopilot_object_configs
    combined_scenarios = experiment_folders + autopilot_folders

    for i in range(len(experiment_folders)):
        print(f'Scenario {i+1}:')
        print_string = ''
        print_string += f'  Experiment: {experiment_folders[i]}\n'
        print_string += f'  Sensor configuration: {object_configs[i]}\n'
        
        print(print_string)

    for i in range(len(autopilot_folders)):
        print(f'Scenario {len(experiment_folders)+i}:')
        print_string = ''
        print_string += f'  Experiment: {autopilot_folders[i]}\n'
        print_string += f'  Sensor configuration: {object_configs[len(experiment_folders)+i]}\n'
        
        print(print_string)


    #print(f'Experiments: {experiment_folders}\nAutopilots: {autopilot_folders}\nObject configs: {object_configs}')
    cont = input('Do you want to continue with the following experiments, autopilots and object config files? [y/n]')
    if cont.lower() not in ['yes', 'y']:
        sys.exit('Exiting...')

    return combined_scenarios, object_configs # experiment_folders, autopilot_folders, odometry_folders, object_configs


class Tmux:
    def __init__(self, session_name='session_test', window_name='session_test'):
        self.session_name=session_name
        self.window_name=window_name

        # init server
        self.server = libtmux.Server()

        # Create session, attach window to it and then divide it to panes
        self.session = self.server.sessions[0] #new_session(session_name=self.session_name, kill_session=False, attach=False)
        self.window = self.session.new_window(attach=True, window_name=self.window_name)
        self.scenario_executor_pane = self.window.split_window(attach=True, vertical=False)
        self.scenario_executor_pane = self.window.panes[1]
        self.scenario_runner_pane = self.window.panes[0]
        self.window.select_layout('even-horizontal')

        #self.session.attach_session()

    def delete_window(self):
        print('deleting tmux window...')
        self.window.kill_window()


    def delete_and_create_new_window(self):
        self.delete_window()

        print('creating new tmux window with panes')
        self.window = self.session.new_window(attach=True, window_name=self.window_name)
        self.scenario_executor_pane = self.window.split_window(attach=True, vertical=False)
        self.scenario_executor_pane = self.window.panes[1]
        self.scenario_runner_pane = self.window.panes[0]
        self.window.select_layout('even-horizontal')

        time.sleep(10.0)


    def send_command(self, pane, command, enter=True):
        pane.select_pane()
        pane.send_keys(command, enter=enter)

    def log_pane(self, pane):
        pane.select_pane()
        print('\n'.join(pane.capture_pane()))

    def attach_session(self):
        self.server.attach_session(self.session_name)



class ExperimentRunner(Node):
    def __init__(self):
        super().__init__('experiment_runner')
        global EXPERIMENT
        global TOWN
        global REPETITIONS
        global ODOMETRY
        global AUTOPILOT
        global EXACT_NAME
        global experiments
        global object_configs

        self.experiments=experiments
        self.object_configs=object_configs
        self.scenario_closed = True

        # init tmux server and session
        self.tmux=Tmux()

        self.tmux.send_command(self.tmux.scenario_runner_pane, 'cd /opt/visual_robot_localization ; colcon build ; cd /opt/carla_vloc_benchmark ; colcon build ;')
        # Wait for the previous command to be done
        time.sleep(7)

        # Create a subscription to ros scenario runner status
        self.subscription = self.create_subscription(
            CarlaScenarioRunnerStatus,
            "/scenario_runner/status",
            self.scenario_finished_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        print(self.experiments)
        print(self.object_configs)
        self.keep_running = True
        self.last_idx = 0
        

        #remaining_executions = False

        #while not remaining_executions:
        #    remaining_executions = self.check_remaining_experiments()
        #    print('All executions are done, move on to the next one.')
        #    if not remaining_executions and len(self.experiments) <= self.last_idx+1:
        #        self.tmux.delete_window()
        #        sys.exit('all experiments done. quitting..')
        #    elif not remaining_executions:
        #        self.last_idx += 1

        #    time.sleep(2)
            
        #self.run_experiment()
        self.find_next_experiment()
        self.run_experiment()
        # Check if there is remaining experiments in the first scenarios
        #if self.check_remaining_experiments():
        #    self.run_experiment()
        #else:
        #    self.last_idx += 1
        #    self.run_experiment()

    def run_experiment(self):
        #if not self.check_remaining_experiments():
        #    self.last_idx+=1
        #    self.close_experiments()
            #self.run_experiment()
        
        town=TOWN

        if TOWN.lower() == 'town10' or TOWN.lower() == 'town10hd':
            town='Town10HD'

        print(f'Running experiment={self.experiments[self.last_idx]}, idx={self.last_idx}')
        reps = REPETITIONS

        scenario_runner_command = f"ros2 launch carla_visual_navigation cli_scenario_runner.launch.py town:={town} " \
                                  f"objects_config:={self.object_configs[self.last_idx]}"
        
        if 'autopilot' in self.experiments[self.last_idx]:
            reps = 1

        scenario_executor_command = f"ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:={self.experiments[self.last_idx]} repetitions:={reps}"
        
        print(f'\n\nRunning following commands:\n - Pane 1: {scenario_runner_command}\n - Pane 2: {scenario_executor_command}\n\n')

        self.tmux.send_command(pane=self.tmux.scenario_runner_pane, command=scenario_runner_command)


        # Wait before starting the executor pane, so that runner has time to init everything
        time.sleep(15.0)
        self.tmux.send_command(pane=self.tmux.scenario_executor_pane, command=scenario_executor_command)
        
        self.scenario_closed = False

        pass

    def close_experiments(self):

        print(f'Closing experiment={self.experiments[self.last_idx]}, idx={self.last_idx}')

        self.tmux.send_command(pane=self.tmux.scenario_runner_pane, command='C-c', enter=False)
        #self.tmux.send_command(pane=self.tmux.scenario_executor_pane, command='C-c', enter=False)
        if self.last_idx+1 > len(self.experiments):
            self.keep_running = False
        print(f'Experminet closed, wait 15 sec...')
        time.sleep(15.0)

        self.scenario_closed = True
        #self.tmux.delete_and_create_new_window()
        pass

    def check_remaining_experiments(self):
        current_scenario_path = self.experiments[self.last_idx]
        
        print(f'\nChecking remaining experiments for {current_scenario_path}')

        # Change repetitions based if the experiment is using autopilot
        reps = REPETITIONS
        if 'autopilot' in current_scenario_path:
            reps = 1

        with open((Path(current_scenario_path) / 'parameters.yml'), 'r+') as f:
            experiment_params = yaml.safe_load(f)

        scenario_list = list(Path(current_scenario_path).glob('*.xosc'))
        
        scenario_list.sort()
        
        scenario_list=[{'scenario_path': path, 'remaining_executions': reps} for path in scenario_list]
        log_file_path = experiment_params['log_file_path']
        if os.path.isfile(log_file_path):
            with open(log_file_path, 'r+') as f:
                logs = json.load(f)

            pruned_list=[]
            execution_count=0

            for scenario_dict in scenario_list:
                for executed_scenario in logs:
                    if executed_scenario["scenario_filepath"] == str(scenario_dict['scenario_path']):
                        scenario_dict['remaining_executions'] -= 1
                        execution_count += 1
                if scenario_dict['remaining_executions'] > 0:
                    pruned_list.append(scenario_dict)
                
            remaining_count=0
            for scenario in pruned_list:
                remaining_count += scenario['remaining_executions']
            
            if remaining_count > 0:
                print(f'  - Execution count:        {execution_count}\n  - Remaining executions:        {remaining_count}')
                return True
            print(f'  - Execution count:        {execution_count}\n  - Remaining executions:      {remaining_count}')
            return False

        # There is no log_file in the results, therefore there is remaining executions
        print("No log file found. All executions remaining")
        return True

    def find_next_experiment(self):

        remaining_executions = False

        while not remaining_executions:
            remaining_executions = self.check_remaining_experiments()
            if not remaining_executions and len(self.experiments) <= self.last_idx+1:
                print('All scenarios are done, close everything...')
                self.tmux.delete_window()
                sys.exit('all experiments done. quitting..')
            elif not remaining_executions:
                print('All executions are done, move on to the next one.')
                if not self.scenario_closed:
                    self.close_experiments()
                self.last_idx += 1
                #self.run_experiment()


            time.sleep(2)
            
        #self.run_experiment()




    def scenario_finished_callback(self, scenario_status_msg):
        '''
        scenario_status:
            uint8 STOPPED = 0
            uint8 STARTING = 1
            uint8 RUNNING = 2
            uint8 SHUTTINGDOWN = 3
            uint8 ERROR = 4
        '''
        scenario_status = scenario_status_msg.status
        if scenario_status == 0:
            print('scenario status is STOPPED')
            
            self.find_next_experiment()
            
            if self.scenario_closed:
                self.run_experiment()
            #remaining_executions=False

            #while not remaining_executions:
            #    remaining_executions=self.check_remaining_experiments()

            #    if not remaining_executions and len(self.experiments) <= self.last_idx+1:
            #        print('All scenarios are done, stopping execution...')
            #        self.tmux.delete_window()
            #        sys.exit('all experiments done. quitting..')
            #    elif not remaining_executions:
            #        print('Move on to the next scenario...')
            #        self.close_experiments()
            #        self.last_idx += 1
            #        self.run_experiment()

            #    time.sleep(2)

        elif scenario_status == 1:
            print('Scenario status is STARTING')

        elif scenario_status == 2:
            print('Scenario status is RUNNING')

        elif scenario_status == 3:
            print('Scenario status is SHUTTING DOWN')
            #self.close_experiments()
            
            # If the scenario status is shutting down, then lets make sure that everything is closing
            self.close_experiments()

            if self.scenario_closed:
                self.find_next_experiment()
                self.run_experiment()
            #if not self.check_remaining_experiments():
            #    self.last_idx+=1
            #elif not self.check_remaining_experiments() and len(self.experiments) <= self.last_idx+1:
            #    self.tmux.delete_window()
            #    sys.exit('All experiments done. quitting...')


            #self.run_experiment()

        elif scenario_status == 4:
            print('Scenario satus is ERROR')
            #self.close_experiments()
            
            # If the scenario status contains an error, then lets make sure that everything is closing
            self.close_experiments()
            
            if self.scenario_closed:    
                self.find_next_experiment()
                self.run_experiment()
            #if not self.check_remaining_experiments():
            #    self.last_idx+=1
            #elif not self.check_remaining_experiments() and len(self.experiments) <= self.last_idx+1:
            #    self.tmux.delete_window()
            #    sys.exit('All experiments done. quitting...')

            #self.run_experiment()

        pass


    def spin(self):
        while (rclpy.ok() & self.keep_running):
            rclpy.spin_once(self, timeout_sec = 1.0)
        pass



def main():
    global EXPERIMENT
    global TOWN
    global REPETITIONS
    global ODOMETRY
    global AUTOPILOT
    global EXACT_NAME
    global node
    global experiments
    global object_configs

    try:
        parser = argparse.ArgumentParser(description='Carla Visual localization benchmark experiments')
        subparsers = parser.add_subparsers(title="subcommands", dest="subcommand")

        scenario_parser = subparsers.add_parser("scenario", help="perform scenarios with parameters")
        scenario_parser.add_argument("--experiment", type=str, required=True, help="experiment name or part of the name. e.g., viewpoint_experiment_yaw, viewpoint_experiment_yaw1, illumination_experiment")
        scenario_parser.add_argument("--town", type=str, required=True, default='Town01', help="Name of the town used in experiment.")
        scenario_parser.add_argument("--autopilot", type=str, required=False, default="False", help="run autopilot scenarios at the same time with the autonomous driving experiments.")
        scenario_parser.add_argument("--odometry", type=str, required=False, default="False", help="run odometry scenarios at the same time with the autonomous driving experiments.")
        scenario_parser.add_argument("--repetitions", type=int, required=False, default=5, help="Define number of repetitions for the autonomous driving experiments.")
        scenario_parser.add_argument("--exact-name", type=str, required=False, default="False", help="If the given experiment name is exact or not.")

        args = parser.parse_args()

        EXACT_NAME= True if args.exact_name.lower() == 'true' else False
        EXPERIMENT=args.experiment
        TOWN=args.town
        REPETITIONS=args.repetitions
        ODOMETRY = True if args.odometry.lower() == 'true' else False
        AUTOPILOT = True if args.autopilot.lower() == 'true' else False
        
        
        rclpy.init(args=None)
        
        experiments, object_configs = read_experiments(experiment_path=EXPERIMENT,
                                                                town=TOWN,
                                                                autopilot=AUTOPILOT,
                                                                odometry=ODOMETRY,
                                                                exact_name=EXACT_NAME)

        print(EXPERIMENT, TOWN, REPETITIONS, ODOMETRY, AUTOPILOT, EXACT_NAME)

        # install required packages
        install_and_import('libtmux')

        node = ExperimentRunner()
        node.spin()
    finally:
        print('Something happened, closing experiments and deleting window...\n\n')
        # Cleanup
        rclpy.shutdown()

        print('Press Ctrl + d to close tmux server, then start the experiment runner script again.')


if __name__ == "__main__":
    main()
