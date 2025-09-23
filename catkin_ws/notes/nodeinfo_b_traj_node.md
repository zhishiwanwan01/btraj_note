Node [/b_traj_node]
Publications: 
 * /b_traj_node/check_trajectory [visualization_msgs/Marker]
 * /b_traj_node/corridor_vis [visualization_msgs/MarkerArray]
 * /b_traj_node/expanded_nodes_vis [visualization_msgs/Marker]
 * /b_traj_node/grid_path_vis [visualization_msgs/MarkerArray]
 * /b_traj_node/path_vis [visualization_msgs/MarkerArray]
 * /b_traj_node/stop_trajectory [visualization_msgs/Marker]
 * /b_traj_node/trajectory [quadrotor_msgs/PolynomialTrajectory]
 * /b_traj_node/trajectory_vis [visualization_msgs/Marker]
 * /b_traj_node/vis_map_inflate [sensor_msgs/PointCloud2]
 * /b_traj_node/vis_map_local [sensor_msgs/PointCloud2]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]

Subscriptions: 
 * /odom/fake_odom [nav_msgs/Odometry]
 * /random_forest_sensing/random_forest [sensor_msgs/PointCloud2]
 * /waypoint_generator/waypoints [nav_msgs/Path]

Services: 
 * /b_traj_node/get_loggers
 * /b_traj_node/set_logger_level


contacting node http://btrajSim-Melodic:38475/ ...
Pid: 209
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (48331 - 172.20.0.2:56496) [14]
    * transport: TCPROS
 * topic: /b_traj_node/vis_map_local
    * to: /rvizvisualisation
    * direction: outbound (48331 - 172.20.0.2:56562) [19]
    * transport: TCPROS
 * topic: /b_traj_node/trajectory_vis
    * to: /rvizvisualisation
    * direction: outbound (48331 - 172.20.0.2:56552) [18]
    * transport: TCPROS
 * topic: /b_traj_node/corridor_vis
    * to: /rvizvisualisation
    * direction: outbound (48331 - 172.20.0.2:56522) [27]
    * transport: TCPROS
 * topic: /b_traj_node/path_vis
    * to: /rvizvisualisation
    * direction: outbound (48331 - 172.20.0.2:56594) [20]
    * transport: TCPROS
 * topic: /b_traj_node/grid_path_vis
    * to: /rvizvisualisation
    * direction: outbound (48331 - 172.20.0.2:56588) [30]
    * transport: TCPROS
 * topic: /b_traj_node/expanded_nodes_vis
    * to: /rvizvisualisation
    * direction: outbound (48331 - 172.20.0.2:56578) [29]
    * transport: TCPROS
 * topic: /b_traj_node/check_trajectory
    * to: /rvizvisualisation
    * direction: outbound (48331 - 172.20.0.2:56536) [16]
    * transport: TCPROS
 * topic: /b_traj_node/stop_trajectory
    * to: /rvizvisualisation
    * direction: outbound (48331 - 172.20.0.2:56524) [28]
    * transport: TCPROS
 * topic: /b_traj_node/trajectory
    * to: /b_traj_server
    * direction: outbound (48331 - 172.20.0.2:56512) [12]
    * transport: TCPROS
 * topic: /tf
    * to: /rvizvisualisation
    * direction: outbound (48331 - 172.20.0.2:56520) [26]
    * transport: TCPROS
 * topic: /random_forest_sensing/random_forest
    * to: /random_forest_sensing (http://btrajSim-Melodic:42189/)
    * direction: inbound (56774 - btrajSim-Melodic:43113) [17]
    * transport: TCPROS
 * topic: /odom/fake_odom
    * to: /odom_generator (http://btrajSim-Melodic:39229/)
    * direction: inbound (55676 - btrajSim-Melodic:51437) [15]
    * transport: TCPROS
 * topic: /waypoint_generator/waypoints
    * to: /waypoint_generator (http://btrajSim-Melodic:43451/)
    * direction: inbound (53826 - btrajSim-Melodic:45223) [13]
    * transport: TCPROS

