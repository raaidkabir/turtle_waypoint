# turtle_waypoint
Uses ROS2 libraries in order to create waypoints for the turtle to follow

## How to install
`colcon build`
then
`source install/setup.bash`

## How to Run
`ros2 run turtlesim turtlesim_node` to launch the virtulized turtle 'robot'.

`ros2 service call /load interfaces/srv/Waypoints "{x1: 0, y1: 0,x2: 1, y2: 1, x3: 2, y3: 5 }` loads the points `(0, 0) (1, 1) (2, 5)` 
for the turtle to traverse one by one

Run `ros2 service call /toggle std_srvs/srv/Empty` toggle the turtle between MOVING and STOPPED states
