Node [/b_traj_server]
Publications: 
 * /b_traj_server/desired_acceleration [visualization_msgs/Marker]
 * /b_traj_server/desired_position [geometry_msgs/PoseStamped]
 * /b_traj_server/desired_velocity [visualization_msgs/Marker]
 * /position_cmd [quadrotor_msgs/PositionCommand]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /b_traj_node/trajectory [quadrotor_msgs/PolynomialTrajectory]
 * /odom/fake_odom [nav_msgs/Odometry]

Services: 
 * /b_traj_server/get_loggers
 * /b_traj_server/set_logger_level


contacting node http://btrajSim-Melodic:32947/ ...
Pid: 220
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (48423 - 172.20.0.2:57608) [14]
    * transport: TCPROS
 * topic: /position_cmd
    * to: /odom_generator
    * direction: outbound (48423 - 172.20.0.2:57622) [10]
    * transport: TCPROS
 * topic: /b_traj_server/desired_velocity
    * to: /rvizvisualisation
    * direction: outbound (48423 - 172.20.0.2:57640) [17]
    * transport: TCPROS
 * topic: /b_traj_server/desired_acceleration
    * to: /rvizvisualisation
    * direction: outbound (48423 - 172.20.0.2:57628) [16]
    * transport: TCPROS
 * topic: /odom/fake_odom
    * to: /odom_generator (http://btrajSim-Melodic:39229/)
    * direction: inbound (55678 - btrajSim-Melodic:51437) [12]
    * transport: TCPROS
 * topic: /b_traj_node/trajectory
    * to: /b_traj_node (http://btrajSim-Melodic:38475/)
    * direction: inbound (56512 - btrajSim-Melodic:48331) [15]
    * transport: TCPROS

