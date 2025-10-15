# Laboratorio-5

¡Perfecto, bro! Entiendo. Aquí te envío el contenido organizado para que puedas usarlo en GitHub:

---

# INVESTIGACIÓN: ROS, MoveIt, Gazebo, LIDAR, SLAM y Docker en IoT

## 📋 Resumen Ejecutivo - Primer Punto

Esta investigación cubre las herramientas fundamentales para desarrollo robótico en entornos IoT, incluyendo:
- **ROS (Robot Operating System)**: Framework para desarrollo de software robótico
- **MoveIt**: Framework para manipulación y planificación de movimientos
- **Gazebo**: Simulador de robots 3D
- **Sistemas LIDAR**: Tecnología de mapeo y detección
- **SLAM**: Algoritmos de localización y mapeo simultáneo
- **Docker**: Contenerización para despliegue en IoT

## 🔧 Sección de Códigos - Ejemplos Prácticos

### 🤖 Ejemplo 1: Robot Sencillo con ROS, MoveIt y Gazebo

#### Objetivo
Crear un robot básico con capacidad de movimiento usando ROS, MoveIt y Gazebo.

#### Estructura del Proyecto
```
simple_robot/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   └── simple_robot.urdf
├── launch/
│   └── gazebo.launch
├── config/
│   └── moveit_config.yaml
└── scripts/
    └── controller.py
```

#### Códigos Principales

**1. simple_robot.urdf** - Modelo del robot
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
  </link>
  
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <origin xyz="0 0.3 0" rpy="0 0 0"/>
  </joint>
</robot>
```
*Explicación: Define la estructura física del robot con geometrías básicas y articulaciones.*

**2. gazebo.launch** - Lanzamiento en Gazebo
```xml
<launch>
  <param name="robot_description" 
         command="cat $(find simple_robot)/urdf/simple_robot.urdf" />
  
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="worlds/empty.world"/>
  </include>
  
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
        args="-param robot_description -urdf -model simple_robot" />
</launch>
```
*Explicación: Carga el mundo de Gazebo y spawnea el modelo del robot.*

**3. controller.py** - Controlador de movimiento
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class SimpleController:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz
        
    def move_forward(self, duration=5):
        move_cmd = Twist()
        move_cmd.linear.x = 0.5  # 0.5 m/s
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.cmd_pub.publish(move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('simple_controller')
    controller = SimpleController()
    controller.move_forward(10)  # Avanzar por 10 segundos
```
*Explicación: Publica comandos de velocidad para mover el robot hacia adelante.*

#### Pasos de Ejecución
```bash
# Terminal 1 - Lanzar Gazebo
roslaunch simple_robot gazebo.launch

# Terminal 2 - Ejecutar controlador
rosrun simple_robot controller.py
```

### 🗺️ Ejemplo 2: Robot con LIDAR y SLAM

#### Objetivo
Implementar un robot con capacidad de mapeo y localización usando LIDAR y SLAM.

#### Estructura del Proyecto
```
lidar_slam_robot/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── slam.launch
│   └── navigation.launch
├── config/
│   └── gmapping_params.yaml
└── maps/
    └── saved_map.pgm
```

#### Códigos Principales

**1. slam.launch** - Configuración SLAM con gmapping
```xml
<launch>
  <!-- LIDAR Node -->
  <node name="rplidarNode" pkg="rplidar_ros" type="rplidarNode">
    <param name="serial_port" type="string" value="/dev/ttyUSB0"/>
    <param name="frame_id" type="string" value="laser"/>
  </node>
  
  <!-- SLAM Gmapping -->
  <node name="slam_gmapping" pkg="gmapping" type="slam_gmapping">
    <param name="base_frame" value="base_link"/>
    <param name="odom_frame" value="odom"/>
    <param name="map_update_interval" value="5.0"/>
    <remap from="scan" to="scan"/>
  </node>
  
  <!-- RVIZ Visualization -->
  <node name="rviz" pkg="rviz" type="rviz" 
        args="-d $(find lidar_slam_robot)/config/slam.rviz"/>
</launch>
```
*Explicación: Configura el nodo LIDAR, algoritmo SLAM gmapping y visualización en RVIZ.*

**2. gmapping_params.yaml** - Parámetros del algoritmo SLAM
```yaml
# Parámetros de Gmapping
slam_gmapping:
  # Configuración LIDAR
  maxRange: 12.0
  maxUrange: 10.0
  particles: 80
  
  # Configuración del mapa
  xmin: -10.0
  xmax: 10.0
  ymin: -10.0
  ymax: 10.0
  delta: 0.05
  
  # Parámetros de actualización
  lstep: 0.05
  astep: 0.05
  ogain: 3.0
```
*Explicación: Optimiza los parámetros del algoritmo gmapping para mejor rendimiento.*

**3. navigation.launch** - Navegación autónoma
```xml
<launch>
  <!-- Cargar mapa existente -->
  <node name="map_server" pkg="map_server" type="map_server" 
        args="$(find lidar_slam_robot)/maps/saved_map.yaml"/>
  
  <!-- AMCL Localization -->
  <node name="amcl" pkg="amcl" type="amcl">
    <param name="min_particles" value="100"/>
    <param name="max_particles" value="5000"/>
  </node>
  
  <!-- Move Base Navigation -->
  <node name="move_base" pkg="move_base" type="move_base">
    <rosparam file="$(find lidar_slam_robot)/config/costmap_common_params.yaml" 
              command="load" ns="global_costmap" />
    <rosparam file="$(find lidar_slam_robot)/config/costmap_common_params.yaml" 
              command="load" ns="local_costmap" />
  </node>
</launch>
```
*Explicación: Configura la localización AMCL y navegación con move_base.*

#### Pasos de Ejecución
```bash
# Terminal 1 - SLAM en tiempo real
roslaunch lidar_slam_robot slam.launch

# Terminal 2 - Guardar mapa (después de mapear)
rosrun map_server map_saver -f ~/map

# Terminal 3 - Navegación autónoma
roslaunch lidar_slam_robot navigation.launch
```

### 🐳 Configuración Docker para IoT

**Dockerfile para entorno ROS**
```dockerfile
FROM ros:noetic-robot

# Instalar dependencias
RUN apt-get update && apt-get install -y \
    ros-noetic-gmapping \
    ros-noetic-navigation \
    ros-noetic-rplidar-ros \
    python3-pip

# Crear workspace
WORKDIR /catkin_ws
COPY . /catkin_ws/src/

# Compilar paquetes
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; \
    catkin_make'

# Configurar entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

**entrypoint.sh** - Script de inicialización
```bash
#!/bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Configurar variables de entorno
export ROS_MASTER_URI=http://$ROS_MASTER_IP:11311
export ROS_IP=$HOST_IP

exec "$@"
```

#### Comandos Docker
```bash
# Construir imagen
docker build -t ros-iot-robot .

# Ejecutar contenedor
docker run -it --net=host \
  -e ROS_MASTER_IP=192.168.1.100 \
  -e HOST_IP=192.168.1.101 \
  ros-iot-robot
```

---
# Resultados



¿Te parece bien esta estructura, bro? Con esto ya tienes todo organizado para GitHub y los ejemplos listos para implementar.
