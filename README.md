# Escape Room ROS2 — Technical Project

Progetto di simulazione per il corso di Robotics Lab: due robot cooperano in un ambiente Gazebo per risolvere un “escape room” corridor.  
Il robot mobile **fra2mo** naviga nel corridoio, riconosce un **ArUco marker** sulla porta bloccata e invia l’ID a **armando** (manipolatore), che inserisce il PIN corrispondente su un tastierino. Una volta inserito il PIN, la porta scorre e fra2mo può uscire.

## Obiettivo
- Navigazione autonoma con avoidance ostacoli
- Rilevamento ArUco e conversione ID → PIN
- Inserimento PIN tramite manipolatore
- Apertura porta tramite giunto prismatico in Gazebo

## Architettura ad alto livello
1. **fra2mo** naviga fino alla zona porta (map + Nav2)
2. Rileva l’**ArUco marker** e pubblica l’ID
3. **aruco_to_pin_bridge** invia il PIN su `/pin_code`
4. **armando** digita il PIN sul tastierino
5. **door_controller** apre la porta su `/door_slide/cmd_pos`

## Struttura repo
- `escape_room/`
- `aruco_ros/` (detection ArUco)
- `m-explore-ros2/` (esplorazione autonoma, opzionale)

Dentro `escape_room`:
- `worlds/escape_room.sdf`: ambiente Gazebo
- `models/`, `meshes/`: modelli e risorse
- `urdf/`: robot fra2mo e armando
- `maps/escape_room_map.yaml`: mappa salvata
- `launch/`: launch files principali
- `scripts/`: nodi Python
- `src/`: nodi C++

## Requisiti
- ROS 2 (con `colcon`, `xacro`, `robot_state_publisher`, `joint_state_publisher`)
- Gazebo + bridge ROS↔Gazebo (`ros_gz_sim`, `ros_gz_bridge`, `ros_ign_bridge`)
- Nav2 (`nav2_bringup`, `nav2_simple_commander`)
- `slam_toolbox`, `explore_lite`
- `aruco_ros`, `aruco_msgs`

## Build
Da root della workspace:
```bash
colcon build --symlink-install
source install/setup.bash
