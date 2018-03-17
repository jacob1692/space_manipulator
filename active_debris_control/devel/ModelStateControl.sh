rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: space_robot, pose: { position: { x: 2, y: 1, z: 1.5 }, orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.6 } }, twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0} }, reference_frame: world }'
echo "ready"
