
import json
from pathlib import Path
import numpy as np
import math
from copy import deepcopy
import os

import matplotlib.pyplot as plt
import seaborn
import itertools
from copy import deepcopy

def main():
    
    # Group by:
    # - Town
    #  - odometry
    #  - Main category (illumination, viewpoint, weather...)
    #    - sub category (yaw, z, roll etc.)
    #      - experiment & autopilot files
    experiment_files_grouped = {}
    
    result_dir = '/results/wacv/files'

    # Get all experiment results (experiments and autopilots) and odometries
    experiment_files = [f for f in os.listdir(result_dir) if os.path.isfile(os.path.join(result_dir, f)) and 'odometry' not in f]
    odometry_files = [f for f in os.listdir(result_dir) if os.path.isfile(os.path.join(result_dir, f)) and 'odometry' in f]
    
    # Loop through odometry files and find one for each town
    for filename in odometry_files:
        town = [x for x in filename.split('_') if 'town' in x][0]
        save_dir = os.path.join(result_dir, town.lower(), 'odometry')
        Path(save_dir).mkdir(parents=True, exist_ok=True)

        odometry_file_path = os.path.join(result_dir, filename)
        odometry_filename = filename

        # Get odometry results and save
        odometry_aggregate = aggregate_results(odometry_file_path)
        with open(os.path.join(save_dir, odometry_filename.replace('.json', '_grouped_by_scenario.json')), 'w') as f:
            json.dump(odometry_aggregate, f, indent=4)
        
        # Store the results in the main dictionary for later use
        odometry_failure_rate = odometry_aggregate["combination_000.xosc"]["scenario_results"]["crashes_per_km"]
        odometry_fragmentation = odometry_aggregate["combination_000.xosc"]["scenario_results"]["crash_fragmentation"]
        experiment_files_grouped[town] = {'odometry': {'failure_rate': odometry_failure_rate, 'fragmentation': odometry_fragmentation, 'aggregate': odometry_aggregate } }
    
    # Define special key values for viewpoint experiments (angles and heights), used for plotting
    yaw_vals = [0, 90, 67.5, 45, 22.5, -22.5, -45, -67.5, -90]
    z_vals = [0, 2, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 16]
    town_plotnames = {'town01': 'Town01', 'town10': 'Town10'}
   
    # Loop through all the experiment files to build the main dictionary
    for filename in experiment_files:
        # Filenames are in order: main category_experiment_sub category_town OR main category_experiment_town
        # Therefore, town is always idx: 2 or 3, main category is idx 1 and sub category 2

        splitted_filename=filename.replace('.json', '').split('_')
        main_category = splitted_filename[0]
        
        # init vars fpr the dictionary
        sub_category = None
        town = None
        autopilot = False
        method = None
        sub_cat_idx = None
        experiment_val = None
        
        # check if the file is for autopilot or experiment
        if 'autopilot' in filename:
            autopilot = True

            # the experiment name is always 5 long if there is main category, town, sub category and autopilot (e.g., viewpoint_experiment_yaw1_town01_autopilot)
            if len(splitted_filename) == 5:
                town=splitted_filename[3]
                sub_category=splitted_filename[2]
                method=''.join(i for i in sub_category if not i.isdigit())
                sub_cat_idx=int(''.join(i for i in sub_category if i.isdigit()))
                
                # Use special key values (e.g. viewpoint_experiment_zpitch1_autopilot, zpitch1 is the sub category, 1 in the end is the index to get the correct value from the list)
                if 'yaw' in method.lower():
                    experiment_val = yaw_vals[sub_cat_idx]
                elif 'pitch' in method.lower():
                    experiment_val = z_vals[sub_cat_idx]
            
            # The experiment name will be 4 long if there is a main category, town, and autopilot
            elif len(splitted_filename) == 4:
                town=splitted_filename[2]

        # Experiment files
        else:
            # File name is always 4 long if it contains main category, town and subcategory (e.g., viewpoint_experiment_yaw1_town01)
            if len(splitted_filename) == 4:
                town=splitted_filename[3]
                sub_category=splitted_filename[2]


                method=''.join(i for i in sub_category if not i.isdigit())
                sub_cat_idx=int(''.join(i for i in sub_category if i.isdigit()))
                
                if 'yaw' in method.lower():
                    experiment_val = yaw_vals[sub_cat_idx]
                elif 'pitch' in method.lower():
                    experiment_val = z_vals[sub_cat_idx]
            # Name is always 3 long if it contains main category and town (e.g., illumination_experiment_town01)
            elif len(splitted_filename) == 3:
                town=splitted_filename[2]
        

        # Define the dictionary items
        dict_vals = {'file_path': os.path.join(result_dir, filename), 'filename': filename, 'experiment_idx': sub_cat_idx, 'experiment_val': experiment_val, 'scenarios': None}    
        empty_dict_vals = {'file_path': None, 'filename': None, 'experiment_idx': None, 'experiment_val': None, 'scenarios': None }

        # Init experiment_files_grouped dict
        if town in experiment_files_grouped.keys():
            # init main category sub dict if not found
            if main_category in experiment_files_grouped[town].keys():
                # init method to the dict if not found
                if method is not None and method in experiment_files_grouped[town][main_category].keys():
                    # init sub category if not found
                    if sub_category in experiment_files_grouped[town][main_category][method].keys():
                        # add data if all keys were found
                        if autopilot:
                            experiment_files_grouped[town][main_category][method][sub_category]['autopilot'] = dict_vals
                        else:
                            experiment_files_grouped[town][main_category][method][sub_category]['experiment'] = dict_vals
                    else:
    
                        if autopilot:
                            experiment_files_grouped[town][main_category][method][sub_category] = { 'autopilot': dict_vals, 'experiment': empty_dict_vals }
                        else:
                            experiment_files_grouped[town][main_category][method][sub_category] = { 'autopilot': empty_dict_vals, 'experiment': dict_vals }
                
                # Name contains e.g., "yaw", viewpoint_experiment_yaw1_town01
                elif method is not None:

                    if autopilot:
                        experiment_files_grouped[town][main_category][method] = { sub_category: { 'autopilot': dict_vals, 'experiment': empty_dict_vals } }
                    else:
                        experiment_files_grouped[town][main_category][method] = { sub_category: { 'autopilot': empty_dict_vals, 'experiment': dict_vals } }
                # Name doesnt contain method, so it must be something like "illumination_experiment_town01"
                elif method is None:
                    if autopilot:
                        experiment_files_grouped[town][main_category]['autopilot'] = dict_vals
                    else:
                        experiment_files_grouped[town][main_category]['experiment'] = dict_vals
            
            # No method, e.g., illumination_experiment_town01
            elif method is None:
                if autopilot:
                    experiment_files_grouped[town][main_category] = {'autopilot': dict_vals, 'experiment': empty_dict_vals}
                else:
                    experiment_files_grouped[town][main_category] = {'autopilot': empty_dict_vals, 'experiment': dict_vals}
            # Contains method, e.g., viewpoint_experiment_yaw1_town1
            elif method is not None:
                if autopilot:
                    experiment_files_grouped[town][main_category] = { method: { sub_category: { 'autopilot': dict_vals, 'experiment': empty_dict_vals } } }
                else:
                    experiment_files_grouped[town][main_category] = { method: { sub_category: { 'autopilot': empty_dict_vals, 'experiment': dict_vals } } }
        # Name contains method
        elif town is not None and method is not None:
            if autopilot:
                experiment_files_grouped[town] = { main_category: { method: { sub_category: { 'autopilot': dict_vals, 'experiment': empty_dict_vals } } } }
            else:
                experiment_files_grouped[town] = { main_category: { method: { sub_category: { 'autopilot': empty_dict_vals, 'experiment': dict_vals } } } }
        # Name doesnt contain method
        elif town is not None and method is None:

            if autopilot:
                experiment_files_grouped[town] = { main_category: { 'autopilot': dict_vals, 'experiment': empty_dict_vals } }
            else:
                experiment_files_grouped[town] = { main_category: { 'autopilot': empty_dict_vals, 'experiment': dict_vals } }
    # Initialization of the dictionary is finished


    # Loop through the dictionary to fill the scenario results
    for town in experiment_files_grouped.keys():

        if town == 'town01':
            exclude_legend = False
            exclude_ylabel = False
        else:
            exclude_legend = True
            exclude_ylabel = True

        # Odometry is required, so if its not present, move on
        if 'odometry' not in experiment_files_grouped[town].keys():
            continue
        for main_category in experiment_files_grouped[town].keys():

            # Skip odometry category, since results for it has been acquired
            if main_category == 'odometry':
                continue
            
            # If autopilot is in the keys, it means we are processing file e.g., illumination_experiment_town01 which doesnt contain methods or subcategories etc.
            if 'autopilot' in experiment_files_grouped[town][main_category].keys():

                # Move on if experiment or autopilot does not have result files
                if experiment_files_grouped[town][main_category]['experiment']['filename'] == None:
                    continue

                print(experiment_files_grouped[town][main_category]['experiment'])
                
                # Create dirs and init vars
                save_dir = os.path.join(result_dir, town, main_category, experiment_files_grouped[town][main_category]['experiment']['filename'].replace('.json', ''))
                plot_savedir = os.path.join(save_dir, 'plots')
                
                Path(plot_savedir).mkdir(parents=True, exist_ok=True)
                
                autopilot_filepath = experiment_files_grouped[town][main_category]['autopilot']['file_path']
                experiment_filepath = experiment_files_grouped[town][main_category]['experiment']['file_path']

                odometry_failure_rate = experiment_files_grouped[town]['odometry']['failure_rate']
                odometry_fragmentation = experiment_files_grouped[town]['odometry']['fragmentation']
                
                # Process files and get results
                aggregate, by_method = process_log_file(experiment_filepath,
                                                        save_dir=save_dir,
                                                        autopilot=False,
                                                        plot_kvalues=True,
                                                        odometry_failure_rate=odometry_failure_rate,
                                                        create_latex=True,
                                                        latex_accuracies=False,
                                                        latex_failurerates=True)
           
                aggregate_autopilot, by_method_autopilot = process_log_file(autopilot_filepath,
                                                                            save_dir=save_dir,
                                                                            autopilot=False,
                                                                            plot_kvalues=False,
                                                                            odometry_failure_rate=0,
                                                                            create_latex=True,
                                                                            latex_accuracies=True,
                                                                            latex_failurerates=False)
                
                # Store results in the dictionary for the later use
                experiment_files_grouped[town][main_category]['autopilot']['scenarios'] = aggregate_autopilot
                experiment_files_grouped[town][main_category]['experiment']['scenarios'] = aggregate
                
                # Create plots
                create_failurerate_recall_plots(by_method_autopilot, by_method, odometry_failure_rate, odometry_fragmentation, plot_savedir, town_plotnames[town], exclude_ylabel=exclude_ylabel, exclude_legend=exclude_legend)
                #plot_crash_locations(aggregate, plot_savedir)
                savepath=os.path.join(plot_savedir, 'delay_plot.png')
                create_computation_delay_plots(by_method_autopilot, savepath)
                
                continue
            # Process files which containes methods and subcategories, e.g., viewpoint_experiment_yaw1_town01
            for method in experiment_files_grouped[town][main_category].keys():
                for sub_category in experiment_files_grouped[town][main_category][method].keys():
                    
                    # Move on if experiment or autopilot doesnt have result file
                    if experiment_files_grouped[town][main_category][method][sub_category]['experiment']['filename'] == None or experiment_files_grouped[town][main_category][method][sub_category]['autopilot']['filename'] == None:
                        continue
                    
                    print(town, main_category, method, sub_category)

                    # Create dirs and init vars
                    save_dir=os.path.join(result_dir, town, main_category, method, experiment_files_grouped[town][main_category][method][sub_category]['experiment']['filename'].replace('.json', ''))
                    plot_savedir=os.path.join(save_dir, 'plots')
                    
                    Path(plot_savedir).mkdir(parents=True, exist_ok=True)


                    autopilot_filepath = experiment_files_grouped[town][main_category][method][sub_category]['autopilot']['file_path']
                    experiment_filepath = experiment_files_grouped[town][main_category][method][sub_category]['experiment']['file_path']
                
                    odometry_failure_rate = experiment_files_grouped[town]['odometry']['failure_rate']
                    odometry_fragmentation = experiment_files_grouped[town]['odometry']['fragmentation']
                    
                    # Get results
                    aggregate, by_method = process_log_file(experiment_filepath,
                                                            save_dir=save_dir,
                                                            autopilot=False,
                                                            plot_kvalues=True,
                                                            odometry_failure_rate=odometry_failure_rate,
                                                            create_latex=True,
                                                            latex_accuracies=False,
                                                            latex_failurerates=True)
           
                    aggregate_autopilot, by_method_autopilot = process_log_file(autopilot_filepath,
                                                                                save_dir=save_dir,
                                                                                autopilot=False,
                                                                                plot_kvalues=False,
                                                                                odometry_failure_rate=0,
                                                                                create_latex=True,
                                                                                latex_accuracies=True,
                                                                                latex_failurerates=False)
                    
                    # Save results in the dict
                    experiment_files_grouped[town][main_category][method][sub_category]['autopilot']['scenarios'] = aggregate_autopilot
                    experiment_files_grouped[town][main_category][method][sub_category]['experiment']['scenarios'] = aggregate
                    
                    # Create plots
                    #create_failurerate_recall_plots(by_method_autopilot, by_method, odometry_failure_rate, odometry_fragmentation, plot_savedir, town_plotnames[town], exclude_ylabel=exclude_ylabel, exclude_legend=exclude_legend)
                    #plot_crash_locations(aggregate, plot_savedir)
                    #savepath=os.path.join(plot_savedir, 'delay_plot.png')
                    #create_computation_delay_plots(by_method_autopilot, savepath)

    # Process viewpoint experiments
    for town in experiment_files_grouped.keys():

        # Odometry is required so only move on if its present
        if 'odometry' not in experiment_files_grouped[town].keys():
            continue
        
        for main_category in experiment_files_grouped[town].keys():
            
            # Do not process odometries itself, its just required for the plots etc.
            if 'odometry' == main_category :
                continue

            for method in experiment_files_grouped[town][main_category].keys():
                
                # if keys contains autopilot it means we are not processing viewpoint experiments or any other stuff which has method included in the name
                if 'autopilot' in experiment_files_grouped[town][main_category].keys():
                    continue
                
                # Collect all the sub categories into new dict, e.g., yaw1, yaw2, yaw3, etc... to the same dict
                experiment_files_grouped_by_method={}

                for sub_category in experiment_files_grouped[town][main_category][method].keys():
                    experiment_scenarios = experiment_files_grouped[town][main_category][method][sub_category]['experiment']['scenarios']
                    autopilot_scenarios = experiment_files_grouped[town][main_category][method][sub_category]['autopilot']['scenarios']

                    dict_key = experiment_files_grouped[town][main_category][method][sub_category]['experiment']['experiment_val']
                    experiment_files_grouped_by_method[dict_key] = {'autopilot': autopilot_scenarios, 'experiment': experiment_scenarios}
                
                # Add benchmark results (0 angle or 0 height), in this case it is illumination result
                experiment_files_grouped_by_method[0] = {'autopilot': experiment_files_grouped[town]['illumination']['autopilot']['scenarios'], 'experiment': experiment_files_grouped[town]['illumination']['experiment']['scenarios']}

                # Define here different plotting funcitionality for experiments if needed.
                if main_category == 'viewpoint':
                    
                    # get grouped results
                    by_method, by_method_autopilot = group_by_method_and_viewpoint(experiment_files_grouped_by_method)
                    
                    # Create a plot and results for each kvalue
                    for kvalue in by_method.keys():
                        print(f'{kvalue}: {method}')

                        # Init vars and create dirs
                        kvalue_filename = str(int(kvalue))
                        by_method_save = deepcopy(by_method[kvalue])
                        by_method_autopilot_save = deepcopy(by_method_autopilot[kvalue])

                        save_dir = os.path.join(result_dir, town, main_category, method)
                        Path(save_dir).mkdir(parents=True, exist_ok=True)
                        
                        # Create result json files
                        with open(os.path.join(save_dir, f'{method}_kval{kvalue_filename}_grouped.json'), 'w') as f:
                            json.dump(by_method_save, f, indent=4)
        
                        with open(os.path.join(save_dir, f'{method}_kval{kvalue_filename}_grouped_autopilot.json'), 'w') as f:
                            json.dump(by_method_autopilot_save, f, indent=4)
                        
                        # Get odometry results
                        odometry_failure_rate = experiment_files_grouped[town]['odometry']['failure_rate']
                        odometry_fragmentation = experiment_files_grouped[town]['odometry']['fragmentation']
                        
                        # Create and save plots
                        Path(os.path.join(save_dir, 'plots', f'kvalue_{kvalue_filename}')).mkdir(parents=True, exist_ok=True)
                        plot_savedir=os.path.join(save_dir, 'plots', f'kvalue_{kvalue_filename}')

                        if method == 'zpitch':
                            xaxis_label = 'z'
                        else:
                            xaxis_label = method
                        create_failurerate_recall_plots(by_method_autopilot[kvalue], by_method[kvalue], odometry_failure_rate, odometry_fragmentation, plot_savedir, town_plotnames[town], exclude_ylabel=True, exclude_legend=True, plot_label=f'{xaxis_label}')
                        #savepath=os.path.join(plot_savedir, 'delay_plot.png')
                        #create_computation_delay_plots(by_method_autopilot[kvalue], savepath)

                        # Create latex strings for the paper
                        # Failure rates
                        create_latex_string(by_method_save, '/dummy/kval_{}.json'.format(kvalue), save_dir,  False, True)
                        # Recall rates
                        create_latex_string(by_method_autopilot_save, '/dummy/kval_{}.json'.format(kvalue), save_dir,  True, False)


                
    return

def plot_crash_locations(scenario_runs, savedir):

    x_offset = 0
    y_offset = 0
    scale = 0.99
    
    prev_kval=None
    crashes=[]
    savedir = os.path.join(savedir, 'crashplots')

    Path(savedir).mkdir(parents=True, exist_ok=True)

    for scenario_name, scenario in scenario_runs.items():
        
        crash_locations_x =  []
        crash_locations_y = []

        for crash_location in scenario['scenario_results']['crash_locations']:
                crash_locations_x.append((crash_location['x'])*scale + x_offset)
                crash_locations_y.append((crash_location['y'])*scale + y_offset)

        
        kvalue = abs(math.log(float(scenario['scenario_params']['sun_intensity']), 1/2))

        specifier = scenario['scenario_params']['global_extractor_name'] + '_' + scenario['scenario_params']['local_extractor_name'] + '_k' + str(kvalue)
        specifier = specifier.replace('dir', 'Ap-GeM')
        savepath = savedir + '/' + specifier + '.png'

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


def process_log_file(log_file_path, save_dir, autopilot, plot_kvalues, odometry_failure_rate, create_latex, latex_accuracies, latex_failurerates):
    
    _, experiment_filename=os.path.split(log_file_path)

    if autopilot:
        scenario_aggregate_results = aggregate_results_autopilot_accuracy_test(log_file_path)
        with open(os.path.join(save_dir, experiment_filename.replace('.json', '_grouped_by_scenario.json')), 'w') as f:
            json.dump( scenario_aggregate_results, f,indent=4)
    else:
        scenario_aggregate_results = aggregate_results(log_file_path)
        with open(os.path.join(save_dir, experiment_filename.replace('.json', '_grouped_by_scenario.json')), 'w') as f:
            json.dump( scenario_aggregate_results, f,indent=4)

    # # Result not valid for scenarios with negative sun elevation angle or intensity
    scenario_results_by_method = group_by_method_and_kvalue(scenario_aggregate_results)
    scenarios_grouped_by_method_save = deepcopy(scenario_results_by_method)

    with open(os.path.join(save_dir, experiment_filename.replace('.json', '_grouped_by_method.json')), 'w') as f:
        json.dump( scenarios_grouped_by_method_save, f, indent=4)

    if create_latex:
        create_latex_string(scenarios_grouped_by_method_save, log_file_path, save_dir,  latex_accuracies, latex_failurerates)

    return scenario_aggregate_results, scenario_results_by_method

def create_computation_delay_plots(recallrates_by_method, savepath):

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

def create_failurerate_recall_plots(recallrates_by_method, failurerates_by_method, odometry_failurerate, odometry_fragmentation, savedir, town, exclude_ylabel=False, exclude_legend=False, plot_label='k'):
    
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
        print(f'k values = {kvalues}')

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
        if(len(recall) < len(failurerates)):
            [recall.append(0) for i in range(len(failurerates)-len(recall))]
        
        fig = seaborn.scatterplot(x=failurerates, y=recall, style=markers, hue=colors, s=markersize, palette='colorblind', legend=not(exclude_legend))
        ax = plt.gca()
        line = fig.axvline(odometry_failurerate, label='Wheel odometry')
        if not exclude_legend:
            fig.legend(ncol=2, fontsize=12)
        plt.ylim([-5,107])
        plt.xlim([-1,37])
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
    
    save_label=plot_label

    if ',' in plot_label:
        save_label=plot_label.split(',')[0]
    
    experiment_name=''.join(i for i in save_label if i.isalnum())

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
    plt.xlabel(plot_label)

    plt.savefig(savedir+'/fragmentation_{}_correlation.png'.format(experiment_name))
    plt.close()


    ## failure vs kvalue

    plt.figure(figsize=[7,5], layout='tight')
    # plt.figure(figsize=[14,10], layout='tight')


    fig = seaborn.scatterplot(x=kvaluelist, y=failurerates, style=markers, hue=colors, s=100, palette='colorblind', legend=not(exclude_legend))
    ax = plt.gca()
    
    ymax = np.nanmax(failurerates[failurerates != None]) + 1
    plt.ylim(bottom=-1)
    line = fig.axhline(odometry_failurerate, label='Wheel\nodometry')
    fig.legend(ncol=2, fontsize=15, loc='upper left')


    fig = seaborn.lineplot(x=kvaluelist, y=failurerates, style=markers, hue=colors, dashes=False, palette='colorblind', legend=False)

    plt.title(town,fontsize=18)  
    if not exclude_ylabel:
        plt.ylabel('Failures/1000m', fontsize=20)
    else:
        ax.yaxis.set_ticklabels([])
    plt.xlabel(plot_label, fontsize=20)
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)

    plt.ylim(top=45)


    if exclude_legend:
        ax.get_legend().remove()
    plt.savefig(savedir+'/failurerate_{}_correlation_{}.png'.format(experiment_name, town))
    plt.close()

    return

def create_latex_string(scenarios_grouped_by_method_sorted_save, log_file_path, save_dir, accuracies, failures):
    
    _, experiment_filename=os.path.split(log_file_path)

    if accuracies:
        latex_string = ''
        for method ,v in scenarios_grouped_by_method_sorted_save.items():
            latex_string += method + ' '
            for kvalue, v_ in v.items():
                #if kvalue < 13:
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
               # if kvalue < 13:
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
            save_path = os.path.join(save_dir, experiment_filename.replace('.json', '_latex_accuracies.txt'))
            Path(save_path).write_text(latex_string)

    if failures:
        latex_string = ''
        for method ,v in scenarios_grouped_by_method_sorted_save.items():
            latex_string += method + ' '
            for kvalue, v_ in v.items():
                #if kvalue < 13:
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
            save_path = os.path.join(save_dir, experiment_filename.replace('.json', '_latex_failures.txt'))
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

def group_by_method_and_viewpoint(aggregate_results_list):
    scenarios_grouped_by_method = {}
    scenarios_grouped_by_method_autopilot = {}
    yaw_degs = [90, 67.5, 45, 22.5, 0, -22.5, -45, -67.5, -90]
    zpitch = [2, 4,6,7,8,9,10,11,12,13,15, 17, 18]
    
    print(aggregate_results_list.keys())

    result_keys = sorted(list([key for key in aggregate_results_list.keys()]))
    
    print(result_keys)
    for key in result_keys:
        #key = str(key)
        
        
        if aggregate_results_list[key]['experiment'] is not None and aggregate_results_list[key]['autopilot'] is not None:
            
            experiment_results = list([scenario for _, scenario in aggregate_results_list[key]['experiment'].items()])
            autopilot_results = list([scenario for _, scenario in aggregate_results_list[key]['autopilot'].items()])
            
            if key == 0:
                experiment_results = [scenario for scenario in experiment_results if math.log(float(scenario['scenario_params']['sun_intensity']), 1/2) == 0 or math.log(float(scenario['scenario_params']['sun_intensity']), 1/2) == 5]
                autopilot_results = [scenario for scenario in autopilot_results if math.log(float(scenario['scenario_params']['sun_intensity']), 1/2) == 0 or math.log(float(scenario['scenario_params']['sun_intensity']), 1/2) == 5]
            #if 'yaw' in method.lower():
            #    method_id = str(yaw_degs[i])

            #    if method_id=='0':
            #        experiment_results = list([scenario for _, scenario in illumination_experiment['experiment'].items()])
            #        autopilot_results = list([scenario for _, scenario in illumination_experiment['autopilot'].items()])
            #        experiment_results = [scenario for scenario in experiment_results if math.log(float(scenario['scenario_params']['sun_intensity']), 1/2) == 0 or math.log(float(scenario['scenario_params']['sun_intensity']), 1/2) == 5]
            #        autopilot_results = [scenario for scenario in autopilot_results if math.log(float(scenario['scenario_params']['sun_intensity']), 1/2) == 0 or math.log(float(scenario['scenario_params']['sun_intensity']), 1/2) == 5]
            #    else:
            #        result_keys.pop(0)
            #elif 'pitch' in method.lower():
            #    method_id=str(zpitch[i])

            #    if method_id=='2': 
            #        experiment_results = list([scenario for _, scenario in illumination_experiment['experiment'].items()])
            #        autopilot_results = list([scenario for _, scenario in illumination_experiment['autopilot'].items()])
            #        experiment_results = [scenario for scenario in experiment_results if math.log(float(scenario['scenario_params']['sun_intensity']), 1/2) == 0 or math.log(float(scenario['scenario_params']['sun_intensity']), 1/2) == 5]
            #        autopilot_results = [scenario for scenario in autopilot_results if math.log(float(scenario['scenario_params']['sun_intensity']), 1/2) == 0 or math.log(float(scenario['scenario_params']['sun_intensity']), 1/2) == 5]
            #    else:
            #        result_keys.pop(0)
            #print(method_id)
            
            #i+=1

            # Experiment
            for scenario in list(experiment_results):
                ge = scenario['scenario_params']['global_extractor_name']
                le = scenario['scenario_params']['local_extractor_name']
                lm = scenario['scenario_params']['local_matcher_name']

                id = ge + '+' + le + '+' + lm
                
                try:
                    kvalue = math.log(float(scenario['scenario_params']['sun_intensity']), 1/2)
                    if not kvalue in scenarios_grouped_by_method:
                        scenarios_grouped_by_method[kvalue]={}
                    
                    if not id in scenarios_grouped_by_method[kvalue]:
                        scenarios_grouped_by_method[kvalue][id]={}
                    
                    

                    scenarios_grouped_by_method[kvalue][id][key]=scenario['scenario_results']

                except ValueError:
                    continue

            
            # Autopilot
            for scenario in list(autopilot_results):
                ge = scenario['scenario_params']['global_extractor_name']
                le = scenario['scenario_params']['local_extractor_name']
                lm = scenario['scenario_params']['local_matcher_name']

                id = ge + '+' + le + '+' + lm
                 
                try:
                    kvalue = math.log(float(scenario['scenario_params']['sun_intensity']), 1/2)
                    if not kvalue in scenarios_grouped_by_method_autopilot:
                        scenarios_grouped_by_method_autopilot[kvalue]={}
                    
                    if not id in scenarios_grouped_by_method_autopilot[kvalue]:
                        scenarios_grouped_by_method_autopilot[kvalue][id]={}
                    
                    scenarios_grouped_by_method_autopilot[kvalue][id][key]=scenario['scenario_results']

                except ValueError:
                    continue 

    return scenarios_grouped_by_method, scenarios_grouped_by_method_autopilot




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
