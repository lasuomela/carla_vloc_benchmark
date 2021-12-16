
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from pathlib import Path
import json
from datetime import datetime
import time

from carla_ros_scenario_runner_types.msg import CarlaScenario, CarlaScenarioRunnerStatus
from carla_ros_scenario_runner_types.srv import ExecuteScenario

class ScenarioBatchExecutor(Node):

    def __init__(self):
        super().__init__('scenario_batch_executor')

        self.declare_parameter("scenario_dir")
        scenario_dir = self.get_parameter('scenario_dir').get_parameter_value().string_value

        self.declare_parameter("repetitions")
        repetitions = self.get_parameter('repetitions').get_parameter_value().integer_value

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
        self.scenario_futures = []

        self.keep_running = True
        print('Finished init')

        # Start scenario execution loop by simulating a "scenario finished" message
        # Sleep for 2s to give service client time to initialize
        time.sleep(2)
        self.scenario_finished_callback( CarlaScenarioRunnerStatus(status=0) )

    def scenario_finished_callback(self, scenario_status_msg):
        '''
        uint8 STOPPED = 0
        uint8 STARTING = 1
        uint8 RUNNING = 2
        uint8 SHUTTINGDOWN = 3
        uint8 ERROR = 4
        '''
        scenario_status = scenario_status_msg.status
        if scenario_status == 0:
            time_now = datetime.now()
            if self.current_scenario is not None:
                print("Scenario execution finished, took {:.2f} minutes!".format( (time_now - self.last_execution_request_time).total_seconds() / 60.0 ))
            self.last_execution_request_time = time_now
            self.request_scenario_execution()
        elif scenario_status == 3:
            print('Scenario runner shutting down..')
            self.keep_running = False
        elif scenario_status == 4:
            print('Scenario runner error..')
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
            rclpy.spin_once(self, timeout_sec = 5.0)
            incomplete_futures = []
            for f in self.scenario_futures:
                if f.done():
                    res = f.result()
                    if res:
                        print('Scenario execution request successful')
                    else:
                        print('Scenario execution request failed')
                else:
                    print('Waiting for scenario request execution...')
                    print('Consider restarting the Carla Ros bridge')
                    incomplete_futures.append(f)
            self.scenario_futures = incomplete_futures

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
