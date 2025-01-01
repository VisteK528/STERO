# Zad 1
Do folderu `/src` zostały zaciągnięte pakiety potrzebne do symulacji i sterowania robotem TIAGO.

# Zad 2
Dodanie parametru `--symlink-install` w momencie budowania pakietów ROS z wykorzystaniem komendy `colcon build` powoduje, że tworzy się automatyczne połączenie pomiędzy plikami w folderze `/src` i plikami w folderze `/install` w taki sposób, że zamiany pakietów znajdujących się w folderze `/src` są automatycznie przenoszone do plików z folderu `/install`.

# Zad 3

```bash
ros2 pkg create --build-type ament_cmake --node-name hello_stero_node hello_stero
```

Program wypisał:
```bash
Piotr Patek
Kacper Bielak
```

# Zad 4
```bash
student@gepard:~$ ros2 action info /execute_trajectory
[WARN] [1728545278.893760343] [rcl]: ROS_LOCALHOST_ONLY is deprecated but still honored if it is enabled. Use ROS_AUTOMATIC_DISCOVERY_RANGE and ROS_STATIC_PEERS instead.
[WARN] [1728545278.893798469] [rcl]: 'localhost_only' is enabled, 'automatic_discovery_range' and 'static_peers' will be ignored.
Action: /execute_trajectory
Action clients: 1
    //p/l/a/y/_/m/o/t/i/o/n/2_move_group_node
Action servers: 1
    /move_group
```

# Zad 5
```bash
student@gepard:~$ ros2 action info /execute_trajectory
Action: /execute_trajectory
Action clients: 2
    /rviz
    //p/l/a/y/_/m/o/t/i/o/n/2_move_group_node
Action servers: 1
    /move_group

```

Względem wykonania komendy z pkt. 4 dodany został jako klient węzeł `/rviz`.

# Zad 6

```bash
Service Servers:
    /apply_planning_scene: moveit_msgs/srv/ApplyPlanningScene
    /check_state_validity: moveit_msgs/srv/GetStateValidity
    /clear_octomap: std_srvs/srv/Empty
    /compute_cartesian_path: moveit_msgs/srv/GetCartesianPath
    /compute_fk: moveit_msgs/srv/GetPositionFK
    /compute_ik: moveit_msgs/srv/GetPositionIK
    /get_planner_params: moveit_msgs/srv/GetPlannerParams
    /load_map: moveit_msgs/srv/LoadMap
    /move_group/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /move_group/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /move_group/get_parameters: rcl_interfaces/srv/GetParameters
    /move_group/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /move_group/list_parameters: rcl_interfaces/srv/ListParameters
    /move_group/set_parameters: rcl_interfaces/srv/SetParameters
    /move_group/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /plan_kinematic_path: moveit_msgs/srv/GetMotionPlan
    /query_planner_interface: moveit_msgs/srv/QueryPlannerInterfaces
    /save_map: moveit_msgs/srv/SaveMap
    /set_planner_params: moveit_msgs/srv/SetPlannerParams
  Service Clients:

  Action Servers:
    /execute_trajectory: moveit_msgs/action/ExecuteTrajectory
    /move_action: moveit_msgs/action/MoveGroup
```



# Zad 7
Węzeł `/moveit_simple_controller_manager` jest wtyczką do węzła `/move_group`. Węzeł `/move_group` generuje trajektorię, która następnie jest przekazywana do kontrolerów sprzętowych zarządzanych przez menadżera kontrolerów MoveIT znajdującego się w węźle `/moveit_simple_controller_manager`. Pozwala on na korzystanie z dostępnych zasobów sprzętowych przez pakiet MoveIt.

# Zad 8
Dostępne interfejsy wartości zadanej dla pierwszego przegubu ramienia:
```bash
	arm_1_joint/effort [available] [unclaimed]
	arm_1_joint/position [available] [claimed]
	arm_1_joint/velocity [available] [unclaimed]
```

Dostępne interfejsy stanu dla pierwszego przegubu ramienia:
```bash
    arm_1_joint/effort
	arm_1_joint/position
	arm_1_joint/velocity
```

# Zad 9
state_interfaces:state_start_arm_1_joint/effort -> joint_state_broadcaster:state_end_arm_1_joint/effort

state_interfaces:state_start_arm_1_joint/velocity -> joint_state_broadcaster:state_end_arm_1_joint/velocity

arm_controller:command_start_arm_1_joint/position -> command_interfaces:command_end_arm_1_joint/position

state_interfaces:state_start_arm_1_joint/position -> arm_controller:state_end_arm_1_joint/position

state_interfaces:state_start_arm_1_joint/position -> joint_state_broadcaster:state_end_arm_1_joint/position

# Zad 10
Węzeł odpala się, a końcówka ramienia rusza się poprawnie do pozycji zadanej.

# Zad 11
Wizualizacja przedstawia ramię, które porusza się po trajektorii zaznaczonej zieloną linią.

# Zad 12
Wizualizacja przedstawia ściankę przez którą przenika ramię robota.

# Zad 13
Punkt docelowy ruchu manipulatora zmodyfikowano w tej sposób, aby ominął wygenerowaną przeszkodę.