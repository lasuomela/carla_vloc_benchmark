
import json
from pathlib import Path
import numpy as np
import math
import pprint
from copy import deepcopy

import matplotlib.pyplot as plt
#import seaborn

def main(log_file_path):

    scenario_aggregate_results = aggregate_results(log_file_path)
    with open(log_file_path.replace('.json', '_grouped_by_scenario.json'), 'w') as f:
        json.dump( scenario_aggregate_results, f,indent=4)

    # Result not valid for scenarios with negative sun elevation angle or intensity
    scenario_results_by_method = group_by_method_and_kvalue(scenario_aggregate_results)
    scenarios_grouped_by_method_save = deepcopy(scenario_results_by_method)
    for k,v in scenarios_grouped_by_method_save.items():
        for k_, v_ in v.items():
            for k__, accuracy in v_.items():
                scenarios_grouped_by_method_save[k][k_][k__] = round(accuracy*100,1)
    with open(log_file_path.replace('.json', '_grouped_by_method.json'), 'w') as f:
        json.dump( scenarios_grouped_by_method_save, f,indent=4)

    # # TODO: REMOVE THESE FROM RELEASE VERSION 
    # if scenario_results_by_method:
    #     create_latex_string(scenarios_grouped_by_method_save)
    #     create_kvalue_plots(scenario_results_by_method)


def create_kvalue_plots(scenarios_grouped_by_method_sorted):

    method_results_by_metric = {}
    for method_id, method_results in scenarios_grouped_by_method_sorted.items():
        metrics = list(method_results.items())[0][1].keys()

        results_by_metric = {}
        for metric in metrics:
            metric_results = {}
            for kvalue, results in method_results.items():
                metric_results[kvalue] = results[metric]
            results_by_metric[metric] = metric_results
        method_results_by_metric[method_id] = results_by_metric


    for method, results in method_results_by_metric.items():
        plt.figure(figsize=[18,10])
        for metric, metric_results in results.items():
            seaborn.lineplot(data=metric_results)
            ax = seaborn.scatterplot(data=metric_results)

        plt.title(method.replace('dir', 'Ap-GeM'))
        plt.ylabel('%')
        plt.xlabel('Illumination decrease multiplier, (gallery illumination)*(1/2)^k')
        plt.legend(list(results.keys()))

        #plt.savefig('/results/plots_town10/' + method +'.png')

    plt.figure(figsize=[18,10])
    for method, results in method_results_by_metric.items():
        metric_results = results['success_prct']
        seaborn.lineplot(data=metric_results)
        ax = seaborn.scatterplot(data=metric_results)

    plt.title('Success rates')
    plt.ylabel('%')
    plt.xlabel('Illumination decrease multiplier, (gallery illumination)*(1/2)^k')
    methods = list(method_results_by_metric.keys())
    methods_names = []
    for method in methods:
        methods_names.append( method.replace('dir', 'Ap-GeM'))
    plt.legend(methods_names)

    #plt.savefig('/results/plots_town10/success_rates.png')

def create_latex_string(scenarios_grouped_by_method_sorted_save):

    latex_string = ''
    for method ,v in scenarios_grouped_by_method_sorted_save.items():
        latex_string += method + ' '
        for kvalue, v_ in v.items():
            if kvalue < 13:
                for cat in ["accuracy_0.25m_2deg", "accuracy_0.5m_5deg", "accuracy_5m_10deg"]:

                    most_accurate = True
                    for method_, accuracies in scenarios_grouped_by_method_sorted_save.items():
                        other_accuracy = accuracies[kvalue][cat]
                        if other_accuracy > v_[cat]:
                            most_accurate = False

                    if most_accurate:
                        latex_string += '\\textbf{' + str(v_[cat])  + '}'
                    else:
                        latex_string +=  str(v_[cat])

                    if cat != "accuracy_5m_10deg":
                        latex_string += ' / '
                latex_string += ' & '
        latex_string += '\n'

    latex_string += '\n'
    for method, v in scenarios_grouped_by_method_sorted_save.items():
        latex_string += method + ' '
        kstring = ''
        for kvalue, v_ in v.items():
            if kvalue < 13:
                if kvalue % 2 == 0:
                    kstring += str(kvalue) + ', '
                    for cat in ["accuracy_0.25m_2deg", "accuracy_0.5m_5deg", "accuracy_5m_10deg"]:

                        most_accurate = True
                        for method_, accuracies in scenarios_grouped_by_method_sorted_save.items():
                            other_accuracy = accuracies[kvalue][cat]
                            if other_accuracy > v_[cat]:
                                most_accurate = False

                        if most_accurate:
                            latex_string += '\\textbf{' + str(v_[cat])  + '}'
                        else:
                            latex_string +=  str(v_[cat])

                        if cat != "accuracy_5m_10deg":
                            latex_string += ' / '
                    latex_string += ' & '
        latex_string += '\nk: ' + kstring + '\n'
        latex_string += '\n'

    if latex_string != '':
        Path('/results/latex_accuracies_town10.txt').write_text(latex_string)

    latex_string = ''
    for method ,v in scenarios_grouped_by_method_sorted_save.items():
        latex_string += method + ' '
        for kvalue, v_ in v.items():
            if kvalue < 13:
                for cat in ["success_prct"]:

                    most_accurate = True
                    for method_, accuracies in scenarios_grouped_by_method_sorted_save.items():
                        other_accuracy = accuracies[kvalue][cat]
                        if other_accuracy > v_[cat]:
                            most_accurate = False

                    if most_accurate & (v_[cat] != 0):
                        latex_string += '\\textbf{' + str(v_[cat])  + '}'
                    else:
                        latex_string +=  str(v_[cat])

                latex_string += ' & '
        latex_string += '\n'

    if latex_string != '':
        Path('/results/latex_success_town10.txt').write_text(latex_string)


def group_by_method_and_kvalue(aggregate_results):

    scenarios_grouped_by_method = {}
    for _, scenario in aggregate_results.items():
        ge = scenario['scenario_params']['global_extractor_name']
        le = scenario['scenario_params']['local_extractor_name']
        lm = scenario['scenario_params']['local_matcher_name']

        id = ge + '+' + le + '+' + lm

        try:
            kvalue = math.log(float(scenario['scenario_params']['sun_intensity']), 1/2)
            if not id in scenarios_grouped_by_method:
                scenarios_grouped_by_method[id] = {}

            scenarios_grouped_by_method[id][kvalue] = scenario['scenario_results']
        except ValueError:
            continue

    scenarios_grouped_by_method_sorted = {}
    for combination, results in scenarios_grouped_by_method.items():
        scenarios_grouped_by_method_sorted[combination] = {k: v for k, v in sorted(results.items(), key=lambda item: item[0])}

    return scenarios_grouped_by_method_sorted
    

def aggregate_results(log_file_path):

    with open(log_file_path, 'r+') as f:
        logs = json.load(f)
    
    distinct_scenarios = set()
    for scenario_run in logs:
        distinct_scenarios.add( scenario_run['scenario_filepath'] )
    distinct_scenarios = list(distinct_scenarios)
    distinct_scenarios.sort()

    distinct_scenarios = { str(Path(scenario_name).name): [] for scenario_name in distinct_scenarios}

    for scenario_run in logs:
        distinct_scenarios[ str(Path(scenario_run['scenario_filepath']).name) ].append( scenario_run )

    scenario_names = list(distinct_scenarios.keys())
    scenario_names.sort()

    scenario_aggregate_results = {}
    for scenario_name in scenario_names:
        scenario_runs = distinct_scenarios[scenario_name]

        success_count = 0
        accuracies = {"accuracy_5m_10deg": [],
                      "accuracy_0.5m_5deg": [],
                      "accuracy_0.25m_2deg": []}

        for run in scenario_runs:
            if run['scenario_success']:
                success_count += 1

            for accuracy_category in accuracies.keys():
                accuracies[accuracy_category].append( run[accuracy_category] )

        success_prct = success_count/len(scenario_runs)
        avg_localization_accuracies = { accuracy_category: np.mean( np.array(accuracy_values))
                                        for accuracy_category, accuracy_values in accuracies.items() }

        scenario_aggregate_results[scenario_name] = {'scenario_params': {}, 'scenario_results':{}}

        scenario_params = deepcopy(run)
        del scenario_params['scenario_success']
        for accuracy in accuracies:
            del scenario_params[accuracy]

        scenario_aggregate_results[scenario_name]['scenario_params'] = scenario_params
        scenario_aggregate_results[scenario_name]['scenario_params']['number_of_runs'] = len(scenario_runs) 

        scenario_aggregate_results[scenario_name]['scenario_results']['success_prct'] = success_prct
        for accuracy_category, mean_accuracy in avg_localization_accuracies.items():
            scenario_aggregate_results[scenario_name]['scenario_results'][accuracy_category] = mean_accuracy

    return scenario_aggregate_results
        
if __name__ == '__main__':
    log_file_path = '/results/test_run.json'
    #log_file_path = '/results/town01_streetlight_run.json'
    main(log_file_path)
