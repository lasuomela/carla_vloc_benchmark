
import json
from pathlib import Path
import numpy as np
import math
from copy import deepcopy
import os

import matplotlib.pyplot as plt
import seaborn



def main():

    # Town01
    log_file_path = '/results/odometry_only_town01.json'
    town01_odometry_aggregate = aggregate_results(log_file_path)
    with open(log_file_path.replace('.json', '_grouped_by_scenario.json'), 'w') as f:
        json.dump( town01_odometry_aggregate, f,indent=4)
    town01_odometry_failure_rate = town01_odometry_aggregate["combination_000.xosc"]["scenario_results"]["crashes_per_km"] 
    town01_odometry_fragmentation = town01_odometry_aggregate["combination_000.xosc"]["scenario_results"]["crash_fragmentation"] 


    log_file_path = '/results/town01_illumination_run_navigation.json'
    town01_illumination_aggregate, town01_illumination_by_method = process_log_file(log_file_path,
                                                                                    autopilot = False,
                                                                                    plot_kvalues=True,
                                                                                    odometry_failure_rate=town01_odometry_failure_rate,
                                                                                    create_latex=True,
                                                                                    latex_accuracies=False,
                                                                                    latex_failurerates=True)

    log_file_path = '/results/town01_illumination_run_autopilot.json'
    town01_illumination_aggregate_autopilot, town01_illumination_by_method_autopilot = process_log_file(log_file_path,
                                                                                    autopilot = False,
                                                                                    plot_kvalues=False,
                                                                                    odometry_failure_rate=0,
                                                                                    create_latex=True,
                                                                                    latex_accuracies=True,
                                                                                    latex_failurerates=False)

    plot_savedir = '/results/plots_town01'
    create_failurerate_recall_plots(town01_illumination_by_method_autopilot, town01_illumination_by_method, town01_odometry_failure_rate, town01_odometry_fragmentation, plot_savedir, 'Town01', exclude_legend=True)


    ## Town10

    log_file_path = '/results/odometry_only_town10HD.json'
    town10_odometry_aggregate = aggregate_results(log_file_path)
    with open(log_file_path.replace('.json', '_grouped_by_scenario.json'), 'w') as f:
        json.dump( town10_odometry_aggregate, f,indent=4)
    town10_odometry_failure_rate = town10_odometry_aggregate["combination_000.xosc"]["scenario_results"]["crashes_per_km"] 
    town10_odometry_fragmentation = town10_odometry_aggregate["combination_000.xosc"]["scenario_results"]["crash_fragmentation"] 

    log_file_path = '/results/town10HD_illumination_run_navigation.json'
    town10_illumination_aggregate, town10_illumination_by_method = process_log_file(log_file_path,
                                                                                    autopilot = False,
                                                                                    plot_kvalues=True,
                                                                                    odometry_failure_rate=town10_odometry_failure_rate,
                                                                                    create_latex=True,
                                                                                    latex_accuracies=False,
                                                                                    latex_failurerates=True)

    log_file_path = '/results/town10HD_illumination_run_autopilot.json'
    town10_illumination_aggregate_autopilot, town10_illumination_by_method_autopilot = process_log_file(log_file_path,
                                                                                    autopilot = False,
                                                                                    plot_kvalues=False,
                                                                                    odometry_failure_rate=0,
                                                                                    create_latex=True,
                                                                                    latex_accuracies=True,
                                                                                    latex_failurerates=False)

    plot_savedir = '/results/plots_town10'
    exclude_ylabel = True
    create_failurerate_recall_plots(town10_illumination_by_method_autopilot, town10_illumination_by_method, town10_odometry_failure_rate, town10_odometry_fragmentation, plot_savedir,'Town10', exclude_ylabel)


    create_computation_delay_plots(town10_illumination_by_method_autopilot)

def plot_crash_locations(scenario_runs):

    x_offset = 0
    y_offset = 0
    scale = 1.0

    for scenario_name, scenario in scenario_runs.items():

        crash_locations_x =  []
        crash_locations_y = []


        for crash_location in scenario['scenario_results']['crash_locations']:
                crash_locations_x.append((crash_location['x'])*scale + x_offset)
                crash_locations_y.append((crash_location['y'])*scale + y_offset)


        kvalue = abs(math.log(float(scenario['scenario_params']['sun_intensity']), 1/2))

        if kvalue ==6:
            savedir = '/results/crashplots'
            specifier = scenario['scenario_params']['global_extractor_name'] + '_' + scenario['scenario_params']['local_extractor_name'] + '_k=' + str(kvalue)
            specifier = specifier.replace('dir', 'Ap-GeM')
            savepath = savedir + '/' + specifier + '.png'

            if not os.path.exists(savedir):
                os.makedirs(savedir)

            town01 = plt.imread('/image-gallery/resource/Town01.jpg')

            image_scaler_y = 10
            image_scaler_x = 0
            plt.figure(figsize=[10, 10])
            plt.xlim([-10-image_scaler_x, 360+image_scaler_x])
            plt.ylim([-360-image_scaler_y, 10+image_scaler_y])
            plt.title(specifier)
            plt.plot(crash_locations_x, crash_locations_y, '.', color='r')
            plt.imshow(town01, extent = [-10-image_scaler_x, 360+image_scaler_x, -360-image_scaler_y, 10+image_scaler_y], zorder=0 )

            plt.savefig(savepath)
            plt.close()
            return


def process_log_file(log_file_path, autopilot, plot_kvalues, odometry_failure_rate, create_latex, latex_accuracies, latex_failurerates):

    if autopilot:
        scenario_aggregate_results = aggregate_results_autopilot_accuracy_test(log_file_path)
        with open(log_file_path.replace('.json', '_grouped_by_scenario.json'), 'w') as f:
            json.dump( scenario_aggregate_results, f,indent=4)
    else:
        scenario_aggregate_results = aggregate_results(log_file_path)
        with open(log_file_path.replace('.json', '_grouped_by_scenario.json'), 'w') as f:
            json.dump( scenario_aggregate_results, f,indent=4)

    # # Result not valid for scenarios with negative sun elevation angle or intensity
    scenario_results_by_method = group_by_method_and_kvalue(scenario_aggregate_results)
    scenarios_grouped_by_method_save = deepcopy(scenario_results_by_method)

    with open(log_file_path.replace('.json', '_grouped_by_method.json'), 'w') as f:
        json.dump( scenarios_grouped_by_method_save, f,indent=4)

    if create_latex:
        create_latex_string(scenarios_grouped_by_method_save, log_file_path,  latex_accuracies, latex_failurerates)

    return scenario_aggregate_results, scenario_results_by_method

def create_computation_delay_plots(recallrates_by_method):

    savepath = '/results/plots_town10/delay_plot_sp.png'
    colors = []
    markers = []
    markersize =50

    methods = recallrates_by_method.keys()

    delays = {}
    for method in methods: 
        delays[method] = {'kvalues': [], 'delays': []}

    for method in methods:
        kvalues = recallrates_by_method[method].keys()
        for kvalue in kvalues:
            
            delays[method]['kvalues'].append(kvalue)
            delays[method]['delays'].append(recallrates_by_method[method][kvalue]['avg_vloc_computation_time'])

    fig = plt.figure(figsize=[18,10])
    dels = []
    for method, v in delays.items():
        dels.append(v)
        print(method)
        print('{:.3f}'.format(np.mean(np.array(v['delays']))))
        colors.append(method)
        seaborn.lineplot(x=v['kvalues'],y=v['delays'])
    plt.legend(colors)
    plt.savefig(savepath)
    plt.close()
    return

def create_failurerate_recall_plots(recallrates_by_method, failurerates_by_method, odometry_failurerate, odometry_fragmentation, savedir, town, exclude_ylabel=False, exclude_legend=False):

    failurerates = []
    fragmentations = []
    tracking_errors = []
    kvaluelist = []

    recall_rates = {'accuracy_0.25m_2deg': [],
                    'accuracy_0.5m_5deg': [],
                    'accuracy_5m_10deg': [],
                    'accuracy_20m_180deg': []}

    recall_rates_wild = {'accuracy_0.25m_2deg': [],
                'accuracy_0.5m_5deg': [],
                'accuracy_5m_10deg': [],
                'accuracy_20m_180deg': []}

    colors = []
    markers = []
    markersize =60

    methods = list(failurerates_by_method.keys())
    methods.sort()
    for method in methods:
        kvalues = failurerates_by_method[method].keys()
        for kvalue in kvalues:
            
            if 'dir' in method:
                markers.append('Ap-GeM')
            elif 'netvlad' in method:
                markers.append('NetVLAD')

            if 'sift' in method:
                colors.append('SIFT')
            elif 'superpoint_aachen' in method:
                colors.append('SuperPoint')
            elif 'r2d2' in method:
                colors.append('R2D2')
            elif 'd2net' in method:
                colors.append('D2')

            failurerates.append(failurerates_by_method[method][kvalue]['crashes_per_km'])
            fragmentations.append(failurerates_by_method[method][kvalue]['crash_fragmentation'])
            tracking_errors.append(failurerates_by_method[method][kvalue]['avg_tracking_error'])
            kvaluelist.append(kvalue)
            for category, recall in recallrates_by_method[method][kvalue]['accuracies_vloc'].items():
                recall_rates[category].append(recall*100)
                if category in failurerates_by_method[method][kvalue]['accuracies_vloc']:
                    recall_rates_wild[category].append(failurerates_by_method[method][kvalue]['accuracies_vloc'][category]*100)

    leg = ['']

    for category, recall in recall_rates.items():

        recallstring=category.replace('accuracy_', ' ≤')
        recallstring=recallstring.replace('_', ', ≤')

        plt.figure(figsize=[7,5], layout='tight')
        seaborn.set_style("whitegrid")
        fig = seaborn.scatterplot(x=failurerates, y=recall, style=markers, hue=colors, s=markersize, palette='colorblind', legend=not(exclude_legend))
        ax = plt.gca()
        line = fig.axvline(odometry_failurerate, label='Wheel odometry')
        if not exclude_legend:
            fig.legend(ncol=2, fontsize=12)
        plt.ylim([-5,105])
        plt.xlim([-1,35])
        plt.title(town,fontsize=17)
        if not exclude_ylabel:
            plt.ylabel('Recall @'+recallstring, fontsize=17)
        else:
            ax.yaxis.set_ticklabels([])
        plt.xlabel('Failures/1000m', fontsize=17)
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)


        plt.savefig(savedir+'/failurerate_recall_correlation_{}_{}.png'.format(town,category))
        plt.close()

        plt.figure(figsize=[18,10])
        fig = seaborn.scatterplot(x=fragmentations, y=recall, style=markers, hue=colors, s=markersize, palette='colorblind')
        line = fig.axvline(odometry_fragmentation, label='Wheel odometry')
        fig.legend(ncol=2)
        plt.ylim([-5,105])
        plt.title(str(category))
        plt.ylabel('Recall')
        plt.xlabel('Fragmentation')

        plt.savefig(savedir+'/fragmentation_recall_correlation_{}.png'.format(category))
        plt.close()

        plt.figure(figsize=[18,10])
        fig = seaborn.scatterplot(x=tracking_errors, y=recall, style=markers, hue=colors, s=markersize, palette='colorblind')
        fig.legend(ncol=2)
        plt.ylim([-5,105])
        plt.title(str(category))
        plt.ylabel('Recall')
        plt.xlabel('Avg. Tracking error')

        plt.savefig(savedir+'/trackingerror_recall_correlation_{}.png'.format(category))
        plt.close()

        if recall_rates_wild[category]:
            plt.figure(figsize=[18,10])
            fig = seaborn.scatterplot(x=recall, y=recall_rates_wild[category], style=markers, hue=colors, s=markersize, palette='colorblind')
            fig.legend(ncol=2)
            plt.ylim([-5,105])
            plt.xlim([-5,105])
            plt.title(str(category))
            plt.xlabel('Recall reference')
            plt.ylabel('Recall in the wild')

            plt.savefig(savedir+'/wild_recall_correlation_{}.png'.format(category))
            plt.close()

    plt.figure(figsize=[18,10])
    xmax=np.max(np.array(failurerates))+1
    frag_array = np.array(fragmentations)
    ymax =np.nanmax(frag_array[frag_array != None])+0.05
    fig = seaborn.scatterplot(x=failurerates, y=fragmentations, style=markers, hue=colors, s=markersize, palette='colorblind')
    plt.xlim([0,xmax])
    plt.ylim([0,ymax])
    line = fig.axhline(odometry_fragmentation, label='Wheel odometry', xmax=odometry_failurerate/xmax)

    fig.legend(ncol=2)
    line = fig.axvline(odometry_failurerate, label='Wheel odometry', ymax = odometry_fragmentation/ymax)

    plt.title("Fragmentation vs Failure rate")
    plt.ylabel('Fragmentation')
    plt.xlabel('Failure rates')

    plt.savefig(savedir+'/fragmentation_failurerate_correlation.png'.format(category))
    plt.close()

    ## kvalues

    plt.figure(figsize=[18,10])
    #xmax=np.max(np.array(failurerates))+1
    frag_array = np.array(fragmentations)
    ymax =np.nanmax(frag_array[frag_array != None])+0.05
    fig = seaborn.scatterplot(x=kvaluelist, y=fragmentations, style=markers, hue=colors, s=markersize, palette='colorblind')
    #plt.xlim([0,xmax])
    plt.ylim([0,ymax])
    line = fig.axhline(odometry_fragmentation, label='Wheel odometry')

    fig.legend(ncol=2)

    plt.title("Fragmentation vs kvalue")
    plt.ylabel('Fragmentation')
    plt.xlabel('K-value')

    plt.savefig(savedir+'/fragmentation_kvalue_correlation.png'.format(category))
    plt.close()


    ## failure vs kvalue

    plt.figure(figsize=[7,5], layout='tight')

    fig = seaborn.scatterplot(x=kvaluelist, y=failurerates, style=markers, hue=colors, s=80, palette='colorblind', legend=not(exclude_legend))
    ax = plt.gca()

    plt.ylim([-1,35])
    line = fig.axhline(odometry_failurerate, label='Wheel odometry')
    fig.legend(ncol=2, fontsize=12)

    fig = seaborn.lineplot(x=kvaluelist, y=failurerates, style=markers, hue=colors, dashes=False, palette='colorblind', legend=False)

    plt.title(town,fontsize=17)  
    if not exclude_ylabel:
        plt.ylabel('Failures/1000m', fontsize=17)
    else:
        ax.yaxis.set_ticklabels([])
    plt.xlabel('k-value', fontsize=17)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    if exclude_legend:
        ax.get_legend().remove()
    plt.savefig(savedir+'/failurerate_kvalue_correlation_{}.png'.format(town))
    plt.close()

    return

def create_latex_string(scenarios_grouped_by_method_sorted_save, log_file_path, accuracies, failures):

    if accuracies:
        latex_string = ''
        for method ,v in scenarios_grouped_by_method_sorted_save.items():
            latex_string += method + ' '
            for kvalue, v_ in v.items():
                if kvalue < 13:
                    for cat in ["accuracy_0.25m_2deg", "accuracy_0.5m_5deg", "accuracy_5m_10deg"]:

                        most_accurate = True
                        for method_, accuracies in scenarios_grouped_by_method_sorted_save.items():
                            other_accuracy = accuracies[kvalue]['accuracies_vloc'][cat]
                            if round(other_accuracy*100,1) > round(v_['accuracies_vloc'][cat]*100,1):
                                most_accurate = False

                        if most_accurate:
                            latex_string += '\\textbf{' + '{:.1f}'.format(v_['accuracies_vloc'][cat]*100)  + '}'
                        else:
                            latex_string +=  '{:.1f}'.format(v_['accuracies_vloc'][cat]*100)

                        if cat != "accuracy_5m_10deg":
                            latex_string += ' / '
                    latex_string += ' & '
            latex_string += '\n'

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
                                other_accuracy = accuracies[kvalue]['accuracies_vloc'][cat]
                                if round(other_accuracy*100,1) > round(v_['accuracies_vloc'][cat]*100,1):
                                    most_accurate = False

                            if most_accurate:
                                latex_string += '\\textbf{' + '{:.1f}'.format(v_['accuracies_vloc'][cat]*100) + '}'
                            else:
                                latex_string +=  '{:.1f}'.format(v_['accuracies_vloc'][cat]*100)

                            if cat != "accuracy_5m_10deg":
                                latex_string += ' / '
                        latex_string += ' & '
            latex_string += '\nk: ' + kstring + '\n'
            latex_string += '\n'

        if latex_string != '':
            save_path = log_file_path.replace('.json', '_latex_accuracies.txt')
            Path(save_path).write_text(latex_string)

    if failures:
        latex_string = ''
        for method ,v in scenarios_grouped_by_method_sorted_save.items():
            latex_string += method + ' '
            for kvalue, v_ in v.items():
                if kvalue < 13:
                    for cat in ["crashes_per_km"]:

                        most_accurate = True
                        for method_, accuracies in scenarios_grouped_by_method_sorted_save.items():
                            other_accuracy = accuracies[kvalue][cat]
                            if round(other_accuracy,1) < round(v_[cat],1):
                                most_accurate = False

                        if most_accurate:
                            latex_string += '\\textbf{' + '{:.1f}'.format(v_[cat])  + '}'
                        else:
                            latex_string +=  '{:.1f}'.format(v_[cat])

                    latex_string += ' & '
            latex_string += '\n'

        if latex_string != '':
            save_path = log_file_path.replace('.json', '_latex_failures.txt')
            Path(save_path).write_text(latex_string)


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

        accuracies_vloc = {"accuracy_5m_10deg": [],
                      "accuracy_0.5m_5deg": [],
                      "accuracy_0.25m_2deg": []}

        accuracies_filtered = deepcopy(accuracies_vloc)

        segment_lengths = []
        tracking_errors = []
        segment_end_drifts_distance = []
        segment_end_drifts_angular = []

        crash_locations = []
                
        vloc_computation_times = []
        for run in scenario_runs:
            if 'avg_tracking_error' in run:
                if run['avg_tracking_error'] is not None:
                    tracking_errors.append(run['avg_tracking_error'])

            if 'avg_vloc_computation_time' in run:
                vloc_computation_times.append( run['avg_vloc_computation_time'])

            for segment in run['segments']:


                crash_location = deepcopy(segment['segment_end_location'])
                if crash_location is not None:
                    crash_locations.append(crash_location)

                # Filter segments caused by 'double reinitialization' if present
                if segment['segment_length'] is not None:
                    if segment['segment_length'] > 0.3:
                        segment_lengths.append(segment['segment_length'])

                        segment_end_drifts_distance.append(segment['pose_final_error']['metric'])
                        segment_end_drifts_angular.append(segment['pose_final_error']['angular'])

            for accuracy_category in accuracies_vloc.keys():
                if accuracy_category in run['accuracies_vloc']:
                    accuracies_vloc[accuracy_category].append( run['accuracies_vloc'][accuracy_category] )
                else:
                    accuracies_vloc[accuracy_category].append( 0.0 )
                
                if accuracy_category in run['accuracies_filtered']:
                    accuracies_filtered[accuracy_category].append( run['accuracies_filtered'][accuracy_category] )



        avg_num_crashes, avg_fragmentation = fragmentation_multirun(scenario_runs)

        avg_segment_length = np.mean(np.array(segment_lengths))
        avg_tracking_error = np.mean(np.array(tracking_errors))
        avg_segment_end_drift = np.mean(np.array(segment_end_drifts_distance))
        avg_segment_proportional_drift = np.mean(np.array(segment_end_drifts_distance)/np.array(segment_lengths))
        avg_segment_proportional_drift_angular = np.mean(np.array(segment_end_drifts_angular)/np.array(segment_lengths))

        avg_localization_accuracies_vloc = { accuracy_category: np.mean( np.array(accuracy_values))
                                        for accuracy_category, accuracy_values in accuracies_vloc.items() }

        avg_localization_accuracies_filtered = { accuracy_category: np.mean( np.array(accuracy_values))
                                    for accuracy_category, accuracy_values in accuracies_filtered.items() }

        scenario_aggregate_results[scenario_name] = {'scenario_params': {}, 'scenario_results':{}}

        scenario_params = deepcopy(run)
        del scenario_params['scenario_success']
        del scenario_params['accuracies_vloc']
        del scenario_params['accuracies_filtered']
        del scenario_params['segments']
        del scenario_params['avg_tracking_error']

        scenario_aggregate_results[scenario_name]['scenario_params'] = scenario_params
        scenario_aggregate_results[scenario_name]['scenario_params']['number_of_runs'] = len(scenario_runs) 

        scenario_aggregate_results[scenario_name]['scenario_results']['avg_segment_length'] = avg_segment_length
        scenario_aggregate_results[scenario_name]['scenario_results']['crashes_per_km'] = avg_num_crashes
        scenario_aggregate_results[scenario_name]['scenario_results']['crash_fragmentation'] = avg_fragmentation
        scenario_aggregate_results[scenario_name]['scenario_results']['avg_tracking_error'] = avg_tracking_error
        scenario_aggregate_results[scenario_name]['scenario_results']['accuracies_vloc'] = avg_localization_accuracies_vloc
        scenario_aggregate_results[scenario_name]['scenario_results']['accuracies_filtered'] = avg_localization_accuracies_filtered
        scenario_aggregate_results[scenario_name]['scenario_results']['avg_segment_end_drift'] = avg_segment_end_drift
        scenario_aggregate_results[scenario_name]['scenario_results']['avg_segment_proportional_drift'] = avg_segment_proportional_drift
        scenario_aggregate_results[scenario_name]['scenario_results']['avg_segment_proportional_drift_angular'] = avg_segment_proportional_drift_angular
        scenario_aggregate_results[scenario_name]['scenario_results']['crash_locations'] = crash_locations
        scenario_aggregate_results[scenario_name]['scenario_results']['avg_vloc_computation_time'] = np.mean(np.array(vloc_computation_times))


    return scenario_aggregate_results


def aggregate_results_autopilot_accuracy_test(log_file_path):
    '''
    Legacy to reproduce paper experiments
    '''

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

        accuracies_vloc = {}
        for category in scenario_runs[0]['accuracy_categories_vloc']:
            accuracies_vloc["accuracy_{}m_{}deg".format(category['lim_t'], category['lim_r'])] = []
        
        for run in scenario_runs:

            num_pycolmap_failures = run['pycolmap_failure_count']
            for category in run['accuracy_categories_vloc']:
                try:
                    category_recall = category['true_count'] / (category['true_count'] +category['false_count'] + num_pycolmap_failures)
                except ZeroDivisionError:
                    print('No vloc estimates for run idx {}, a possible failure'.format(run['idx']))
                accuracies_vloc["accuracy_{}m_{}deg".format(category['lim_t'], category['lim_r'])].append(category_recall)

        scenario_aggregate_results[scenario_name] = {'scenario_params': {}, 'scenario_results':{}}

        for k,v in accuracies_vloc.items():
            accuracies_vloc[k] = np.mean(np.array(v))

        scenario_params = deepcopy(run)
        del scenario_params['accuracy_categories_vloc']

        scenario_aggregate_results[scenario_name]['scenario_params'] = scenario_params
        scenario_aggregate_results[scenario_name]['scenario_params']['number_of_runs'] = len(scenario_runs) 

        scenario_aggregate_results[scenario_name]['scenario_results']['accuracies_vloc'] = accuracies_vloc

    return scenario_aggregate_results

def fragmentation_multirun(scenario_runs):
    '''
    Mean fragmentation over multiple repetitions of different scenarios
    '''

    avg_fragmentation = 0
    frag_count = 0
    for run in scenario_runs:
        run_fragmentation = fragmentation( run['segments'], run['path_waypoint_count'] )
        if run_fragmentation is not None:
            avg_fragmentation += run_fragmentation
            frag_count += 1

    if frag_count != 0:
        avg_fragmentation = avg_fragmentation/frag_count
    else:
        avg_fragmentation = None

    num_failures = 0
    for run in scenario_runs:
        num_failures += len(run['segments'])-1
    avg_failures = num_failures/len(scenario_runs)
    avg_failures_per_meter = avg_failures/(run['path_waypoint_count']*2)*1000

    return avg_failures_per_meter, avg_fragmentation
        

def fragmentation(run_segments, total_wp_number):
    '''
    Fragmentation for a single scenario run
    '''

    wp_idxs = []
    for segment in run_segments:
        wp_idx = segment['segment_reached_waypoint_idx']

        if wp_idx is not None:
            # Shift the index to start from 1
            wp_idxs.append(wp_idx+1)
    
    if len(wp_idxs) > 1:

        sum = 0
        for i, wp_idx in enumerate(wp_idxs):

            if i != len(wp_idxs)-1:
                
                delta_f = wp_idxs[i+1] - wp_idx

            else:
                delta_f = wp_idxs[0] + (total_wp_number) - wp_idx

            # If two consecutive reinits to same waypoint, assign weight of 1
            if delta_f == 0:
                delta_f = 1

            sum += -delta_f/total_wp_number * math.log( delta_f/total_wp_number,10)

        fragmentation = 1/math.log(len(wp_idxs), 10) * sum

    else:
        fragmentation = None
    return fragmentation



if __name__ == '__main__':
    main()