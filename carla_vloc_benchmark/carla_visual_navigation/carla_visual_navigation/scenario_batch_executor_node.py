
import rclpy
from rclpy.node import Node

from pathlib import Path
import json
from datetime import datetime
import time

from carla_ros_scenario_runner_types.msg import CarlaScenario, CarlaScenarioRunnerStatus
from carla_ros_scenario_runner_types.srv import ExecuteScenario

from carla_visual_navigation_interfaces.srv import ToggleLocalPlanner
from rclpy.callback_groups import ReentrantCallbackGroup
import carla
import numpy as np
import carla_common.transforms as trans

class ScenarioBatchExecutor(Node):

    '''
    Execute scenarios specified in the files created with scripts/template_scenario_generator.py
    '''

    def __init__(self):
        super().__init__('scenario_batch_executor')


        self.declare_parameter("scenario_dir")
        scenario_dir = self.get_parameter('scenario_dir').get_parameter_value().string_value

        # How many repetitions of each scenario file should be executed
        self.declare_parameter("repetitions")
        repetitions = self.get_parameter('repetitions').get_parameter_value().integer_value

        self.declare_parameter("scenario_timeout", 0.0)
        self.scenario_timeout = self.get_parameter('scenario_timeout').get_parameter_value().double_value

        self.subscription = self.create_subscription(
            CarlaScenarioRunnerStatus,
            "/scenario_runner/status",
            self.scenario_finished_callback,
            10)

        self.current_scenario = None
        self.last_execution_request_time = None 

        with open((Path(scenario_dir) / 'parameters.json'), 'r+') as f:
            experiment_params = json.load(f)

        scenario_list = list(Path(scenario_dir).glob('*.xosc'))
        if not scenario_list:
            raise RuntimeError('No scenario files found in {}'.format(scenario_dir))

        scenario_list.sort()
        self.scenario_list = [{'scenario_path': path, 'remaining_executions': repetitions} for path in scenario_list]

        if 'log_file_path_template' in experiment_params:
            self.scenario_list = prune_scenario_list(experiment_params['log_file_path_template'][0], self.scenario_list)

        self.scenario_repetitions = repetitions

        self.scenario_client = self.create_client(ExecuteScenario, '/scenario_runner/execute_scenario')
        while not self.scenario_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("'Execute scenario' service not available, waiting again...")
        self.scenario_futures = []


        self.keep_running = True

        # If scenario execution timeout is specified
        if self.scenario_timeout != 0.0:
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world=self.client.get_world()
            self.world.wait_for_tick()

            self.ego_vehicle = None
            actors = self.world.get_actors()
            for actor in actors:
                if 'role_name' in actor.attributes:
                    if actor.attributes['role_name'] == 'ego_vehicle':
                        self.ego_vehicle = actor
            
            self.toggle_planner_client = self.create_client(ToggleLocalPlanner, 'toggle_local_planner', callback_group=ReentrantCallbackGroup())
            self.planner_future = None

        print('Finished init')

        self.scenario_finished_callback( CarlaScenarioRunnerStatus(status=0) )


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
            time_now = datetime.now()

            if self.last_execution_request_time is not None:
                execution_duration = (time_now - self.last_execution_request_time).total_seconds()
            else:
                execution_duration = 100

            if execution_duration > 20:
                if self.current_scenario is not None:
                    print("Scenario execution finished, took {:.2f} minutes!".format( (time_now - self.last_execution_request_time).total_seconds() / 60.0 ))
                self.last_execution_request_time = time_now
                self.request_scenario_execution()

        elif scenario_status == 3:
            print('Scenario runner shutting down..')
            if self.scenario_timeout == 0.0:
                self.keep_running = False

        elif scenario_status == 4:
            print('Scenario runner error..')
            if self.scenario_timeout == 0.0:
                self.keep_running = False

    def request_scenario_execution(self):

        first_run = self.current_scenario is None

        if first_run:
            no_executions_remaining = True
        else:
            no_executions_remaining = self.current_scenario['remaining_executions'] == 0

        if (first_run | no_executions_remaining):
            if self.scenario_list:
                self.current_scenario = self.scenario_list.pop(0)
            else:
                print('All scenarios executed!')
                self.keep_running = False
                return

        carla_scenario = CarlaScenario(name='', scenario_file= str(self.current_scenario['scenario_path']))
        scenario_request = ExecuteScenario.Request(scenario = carla_scenario)
        self.scenario_futures.append(self.scenario_client.call_async(scenario_request))

        remaining_count = 0
        for scenario in self.scenario_list:
            remaining_count += scenario['remaining_executions']
        remaining_count += self.current_scenario['remaining_executions'] -1

        print('Made execution request {}/{} for scenario {}, {} executions remaining'.format(self.scenario_repetitions-self.current_scenario['remaining_executions']+1,
                                                                    self.scenario_repetitions,
                                                                    str(self.current_scenario['scenario_path'].name),
                                                                    remaining_count))
        self.current_scenario['remaining_executions'] -= 1

    def spin(self):
        while (rclpy.ok() & self.keep_running):
            rclpy.spin_once(self, timeout_sec = 1.0)

            incomplete_futures = []
            for f in self.scenario_futures:
                if f.done():
                    res = f.result()
                    if res:
                        print('Scenario execution request successful at time {:02d}:{:02d}'.format(datetime.now().hour, datetime.now().minute))
                    else:
                        print('Scenario execution request failed')
                else:
                    print('Waiting for scenario request execution...')
                    incomplete_futures.append(f)
            self.scenario_futures = incomplete_futures


            # Logic to restart scenario if scenario_timeout is specified
            if self.scenario_timeout != 0.0:
                if self.last_execution_request_time is not None:
                    if ( (datetime.now() - self.last_execution_request_time).total_seconds() / 60.0 ) > self.scenario_timeout:

                        if self.planner_future is None:
                            print('Scenario exceeded maximum runtime, triggering again...')
                            req = ToggleLocalPlanner.Request()
                            req.stop = True
                            self.planner_future = self.toggle_planner_client.call_async(req)
                        else:
                            if self.planner_future.done():

                                while np.linalg.norm(trans.carla_velocity_to_numpy_vector( self.ego_vehicle.get_velocity() )) > 0.01:
                                    time.sleep(1.0)
                                
                                self.current_scenario['remaining_executions'] += 1
                                self.scenario_finished_callback( CarlaScenarioRunnerStatus(status=0))
                                self.planner_future = None


def prune_scenario_list(log_file_path, scenario_list):

    if Path(log_file_path).is_file():
        with open(log_file_path, 'r+') as f:
            logs = json.load(f)

        pruned_list = []
        execution_count = 0
        for scenario_dict in scenario_list:
            for executed_scenario in logs:
                if executed_scenario["scenario_filepath"] == str(scenario_dict['scenario_path']):
                    scenario_dict['remaining_executions'] -= 1
                    execution_count += 1
            if scenario_dict['remaining_executions'] > 0:
                pruned_list.append(scenario_dict)

        remaining_count = 0
        for scenario in pruned_list:
            remaining_count += scenario['remaining_executions']

        print('{} executions found in log file, {} remain'.format( execution_count, remaining_count))
        return pruned_list

    print("No previous executions found in log file")
    return scenario_list

def main(args=None):
    rclpy.init(args=args)
    node=ScenarioBatchExecutor()
    node.spin()
    # Cleanup
    rclpy.shutdown()

if __name__ == '__main__':
    main()
