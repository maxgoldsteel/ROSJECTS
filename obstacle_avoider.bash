#!/usr/bin/bash

source "$(dirname "$0")/robot_functions.bash"

echo "Running Naive Obstacle Avoider with Bash Script..."
echo "Press Ctrl+C to Terminate..."
echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"

# Stop robot
set_cmd_vel_linear 0.0
set_cmd_vel_angular 0.0

# Set threshold
threshold=0.35

# Main loop
while :
do
  # Get scan values
  left=$(get_scan_left_ray_range)
  front_left=$(get_scan_front_left_ray_range)
  front=$(get_scan_front_ray_range)
  front_right=$(get_scan_front_right_ray_range)
  right=$(get_scan_right_ray_range)

  # Obstacle checks
  fl_blk=$(echo "${front_left:-10} < $threshold" | bc -l)
  f_blk=$(echo "${front:-10} < $threshold" | bc -l)
  fr_blk=$(echo "${front_right:-10} < $threshold" | bc -l)

  echo "Scan Values: FL=$front_left F=$front FR=$front_right"
  echo "Flags: FL_BLK=$fl_blk F_BLK=$f_blk FR_BLK=$fr_blk"

  # Decision logic
  if [[ "$fl_blk" == "1" && "$f_blk" == "1" && "$fr_blk" == "1" ]]; then
    if (( $(echo "$right < $left" | bc -l) )); then
        echo "Turning left: More space on the left"
        set_cmd_vel_linear 0.0
        set_cmd_vel_angular 0.5
        set_cmd_vel_angular 0.0
    else
        echo "Turning right: More space on the right"
        set_cmd_vel_linear 0.0
        set_cmd_vel_angular -0.5
        set_cmd_vel_angular 0.0
    fi
  
  elif [[ "$fl_blk" == "1" ]] && [[ "$f_blk" == "1" ]] && [[ "$fr_blk" != "1" ]]; then
    echo "Turning right"
    set_cmd_vel_linear 0.0
    set_cmd_vel_angular -0.5
    set_cmd_vel_angular 0.0

  elif [[ "$fl_blk" == "1" ]] && [[ "$f_blk" != "1" ]] && [[ "$fr_blk" == "1" ]]; then
    echo "Moving forward slightly (middle open)"
    time=$(echo "($front - $threshold)/0.1 - 1" | bc -l)
    sleep_time=$(printf "%.2f" "$time")
    set_cmd_vel_linear 0.1
    set_cmd_vel_angular 0.0
    sleep "$sleep_time"
    set_cmd_vel_linear 0.0

  elif [[ "$fl_blk" == "1" ]] && [[ "$f_blk" != "1" ]] && [[ "$fr_blk" != "1" ]]; then
    echo "Front free, turning right"
    set_cmd_vel_linear 0.0
    set_cmd_vel_angular -0.5
    set_cmd_vel_angular 0.0

  elif [[ "$fl_blk" != "1" ]] && [[ "$f_blk" == "1" ]] && [[ "$fr_blk" == "1" ]]; then
    echo "Turning left"
    set_cmd_vel_linear 0.0
    set_cmd_vel_angular 0.5
    set_cmd_vel_angular 0.0

  elif  [[ "$fl_blk" != "1" ]] && [[ "$f_blk" == "1" ]] && [[ "$fr_blk" != "1" ]]; then
    if (( $(echo "$right < $left" | bc -l) )); then
      echo "Front blocked, left open - turning left"
      set_cmd_vel_linear 0.0
      set_cmd_vel_angular 0.5
      set_cmd_vel_angular 0.0
    else
      echo "Front blocked, right open - turning right"
      set_cmd_vel_linear 0.0
      set_cmd_vel_angular -0.5
      set_cmd_vel_angular 0.0
    fi

  elif [[ "$fl_blk" != "1" ]] && [[ "$f_blk" != "1" ]] && [[ "$fr_blk" == "1" ]]; then
    echo "Front and front-left free - turning left"
    set_cmd_vel_linear 0.0
    set_cmd_vel_angular 0.5
    set_cmd_vel_angular 0.0

  else
    echo "Clear path - moving forward"
    time=$(echo "($front - $threshold)/0.1 - 1" | bc -l)
    sleep_time=$(printf "%.2f" "$time")
    set_cmd_vel_linear 0.1
    set_cmd_vel_angular 0.0
    sleep "$sleep_time"
    set_cmd_vel_linear 0.0
  fi

  echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
  sleep 0.1
done

# End of Code
