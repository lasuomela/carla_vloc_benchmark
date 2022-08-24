#!/bin/bash

echo "Publishing logger command..."

while getopts "a:b:c:d:e:f:g:h:i:j:k:l:m:n:o:p:q:r:s:t:u:v:" opt
do
   case "$opt" in
      a ) town_name="$OPTARG" ;;
      b ) global_extractor_name="$OPTARG" ;;
      c ) local_extractor_name="$OPTARG" ;;
      d ) local_matcher_name="$OPTARG" ;;
      e ) gallery_global_descriptor_path="$OPTARG" ;;
     f ) gallery_local_descriptor_path="$OPTARG" ;;
      g ) image_gallery_path="$OPTARG" ;;
      h ) gallery_sfm_path="$OPTARG" ;;
      i ) localization_frequence="$OPTARG" ;;
      j ) top_k_matches="$OPTARG" ;;
      k ) sun_intensity="$OPTARG" ;;
      l ) sun_elevation="$OPTARG" ;;
      m ) precipitation_type="$OPTARG" ;;
      n ) precipitation_intensity="$OPTARG" ;;
      o ) route_entry_name="$OPTARG" ;;
      p ) target_speed="$OPTARG" ;;
      q ) log_file_path="$OPTARG" ;;
      r ) log_results="$OPTARG" ;;
      s ) scenario_success="$OPTARG" ;;
      t ) scenario_filepath="$OPTARG" ;;
      u ) visual_range="$OPTARG" ;;
      v ) cloud_state="$OPTARG" ;;
   esac
done

ros2 service call /log_scenario_results carla_visual_navigation_interfaces/srv/LogScenarioResults "{ scenario_results: {\
 town_name: $town_name, \
 global_extractor_name: $global_extractor_name, \
 local_extractor_name: $local_extractor_name, \
 local_matcher_name: $local_matcher_name, \
 gallery_global_descriptor_path: $gallery_global_descriptor_path, \
 gallery_local_descriptor_path: $gallery_local_descriptor_path, \
 image_gallery_path: $image_gallery_path, \
 gallery_sfm_path: $gallery_sfm_path, \
 localization_frequence: $localization_frequence, \
 top_k_matches: $top_k_matches, \
 sun_intensity: $sun_intensity, \
 sun_elevation: $sun_elevation, \
 precipitation_type: $precipitation_type,
 precipitation_intensity: $precipitation_intensity, \
 visual_range: $visual_range, \
 cloud_state: $cloud_state, \
 route_entry_name: $route_entry_name, \
 target_speed: $target_speed, \
 log_file_path: $log_file_path, \
 log_results: $log_results, \
 scenario_success: $scenario_success, \
 scenario_filepath: $scenario_filepath \
 } \
}"
