#!/usr/bin/bash

# include the functions library (if needed)
source ./robot_functions.bash

echo "Running Robot Statistics with Bash Script..."
echo "Press Ctrl+C to Terminate..."
echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"

# 🛠️ Define a function to extract topic values
get_field_from_topic() {
  topic=$1
  field=$2
  ros2 topic echo -n 1 "$topic" 2>/dev/null | grep "$field" | awk '{print $2}'
}

# 🧭 Initialize starting position for distance tracking
start_x=$(get_field_from_topic /odom position.x)
start_y=$(get_field_from_topic /odom position.y)

# Main loop
while :
do
  # Odometry position
  pos_x=$(get_field_from_topic /odom position.x)
  pos_y=$(get_field_from_topic /odom position.y)
  pos_z=$(get_field_from_topic /odom position.z)

  # Odometry orientation
  ori_r=$(get_field_from_topic /odom orientation.x)
  ori_p=$(get_field_from_topic /odom orientation.y)
  ori_y=$(get_field_from_topic /odom orientation.z)

  # IMU angular velocity
  ang_vel_x=$(get_field_from_topic /imu angular_velocity.x)
  ang_vel_y=$(get_field_from_topic /imu angular_velocity.y)
  ang_vel_z=$(get_field_from_topic /imu angular_velocity.z)

  # IMU linear acceleration
  lin_acc_x=$(get_field_from_topic /imu linear_acceleration.x)
  lin_acc_y=$(get_field_from_topic /imu linear_acceleration.y)
  lin_acc_z=$(get_field_from_topic /imu linear_acceleration.z)

  # Distance covered (2D Euclidean)
  dx=$(echo "$pos_x - $start_x" | bc -l)
  dy=$(echo "$pos_y - $start_y" | bc -l)
  dist=$(echo "sqrt($dx^2 + $dy^2)" | bc -l)

  # Determine direction
  if (( $(echo "$ang_vel_z > 0.1" | bc -l) )); then
    direction="Turning Left"
  elif (( $(echo "$ang_vel_z < -0.1" | bc -l) )); then
    direction="Turning Right"
  elif (( $(echo "$dx < 0.01 && $dy < 0.01" | bc -l) )); then
    direction="Stopped"
  else
    direction="Moving Forward"
  fi

  # Print robot stats
  echo "Distance Covered: $dist m"
  echo "Direction: $direction"
  echo "Odom Position -> x: $pos_x, y: $pos_y, z: $pos_z"
  echo "Odom Orientation -> r: $ori_r, p: $ori_p, y: $ori_y"
  echo "IMU Angular Velocity -> x: $ang_vel_x, y: $ang_vel_y, z: $ang_vel_z"
  echo "IMU Linear Acceleration -> x: $lin_acc_x, y: $lin_acc_y, z: $lin_acc_z"
  echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"

  sleep 0.5
done

# End of Code