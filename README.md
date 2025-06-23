# Manipulador Móvil: Husky + UR5

Este repositorio contiene el **workspace** del trabajo realizado para la implementación de un manipulador móvil. El sistema está compuesto por una plataforma móvil **Husky** y un brazo robótico **UR5**, utilizando **ROS 2** para la comunicación y control. A lo largo del proyecto, se ha trabajado en la integración de la navegación autónoma y la manipulación de objetos mediante el uso de diversos paquetes y herramientas de ROS 2.

## Estructura del Proyecto

El **workspace** se organiza de la siguiente manera:

```plaintext
ros2_ws/
│
├── build/                 
├── install/             
├── log/                  
├── .git/                 
├── src/                 
│   ├── husky_ur5/
│   ├── move_ur5/
│   ├── ur5_moveit_config/
│   ├── robot_navigation/
│   ├── ros2_robotiq_gripper/
│   ├── robot_initial_pose/
│   ├── aws-robomaker-small-warehouse-world/
│   ├── Universal_Robots_ROS2_Gazebo_Simulation/
│   ├── ur_description/
│   └── husky/

### **husky_ur5**

El paquete **`husky_ur5`** integra la plataforma móvil **Husky** con el brazo robótico **UR5** en un sistema unificado. Este paquete proporciona los archivos necesarios para la simulación utilizando **ROS 2** y el entorno simulado en **Gazebo**. A continuación se describen los principales componentes de este paquete:

- **`worlds/`**: Contiene el archivo **`almacen.world`**, que es el mundo simulado utilizado en **Gazebo**. En este mundo se simulan las condiciones de un almacén, con estantes y cajas.

- **`urdf/`**: Incluye el modelo **XACRO** de **Husky** con el **UR5** montado.

- **`param/`**: Contiene los archivos de configuración para **Nav2**.

- **`maps/`**: Almacena el mapa generado mediante **SLAM.

- **`launch/`**: Contiene los archivos **launch** que permiten la configuración y ejecución del sistema:
  - **`gazebo_launch.py`**: El nodo principal para lanzar la simulación en **Gazebo** e inicializar los **controladores** de los robots, permitiendo la interacción entre **Husky** y **UR5**.
  - **`nav2_almacen.launch.py`**: Este archivo configura e implementa **Nav2**, habilitando la **navegación autónoma** en el robot dentro del entorno simulado.
  - **`slam.launch.py`**: Utilizado para generar el mapa utilizando **SLAM**, lo que permite al robot conocer su entorno mientras se desplaza.

- **`config/`**: Esta carpeta contiene varios archivos de configuración:
  - **`husky_ur5_controllers.yaml`**: Archivo de configuración del controlador de **UR5** y **Husky**, especificando cómo se deben controlar los actuadores y motores de ambos robots.
  - Archivos adicionales de configuración de **Nav2**, que ajustan los parámetros de navegación y localización.

- **`behavior_trees/`**: Contiene el **árbol de comportamiento** utilizado por **Nav2**. 

---

Este paquete es fundamental para integrar **Husky** con **UR5** en un sistema completo de navegación y manipulación. Permite la simulación en **Gazebo**, la planificación autónoma con **Nav2**, y el control del **brazo UR5** en el entorno simulado.

### **husky**

El paquete **`husky`** integra la plataforma móvil **Husky** con el brazo **UR5**. Incluye los modelos **URDF** y controladores para la plataforma principalmente, además de varios archivos adicionales para el uso de la plataforma.

---

### **ur_description**

Este paquete contiene los archivos **URDF** y la configuración cinemática del brazo **UR5**. 

---

### **Universal_Robots_ROS2_Gazebo_Simulation**

Proporciona los archivos de configuración necesarios para simular el brazo **UR5** en **Gazebo**, incluyendo la configuración física del brazo.

---

### **ros2_robotiq_gripper**

Contiene los modelos y la configuración para el **gripper Robotiq 2F-85** montado en el **UR5**, permitiendo controlar su apertura y cierre desde **ROS 2** para tareas de manipulación.

---

### **aws-robomaker-small-warehouse-world**

Incluye los modelos usados en un **almacen.world** en **Gazebo**, con estantes y cajas.

### **robot_initial_pose**

Este paquete incluye el nodo **`initial_pose_publisher.py`**, que publica la pose inicial del robot para **Nav2**, permitiendo que el robot comience su navegación desde una posición conocida.

---

### **ur5_moveit_config**

Contiene los archivos de configuración y los nodos de lanzamiento generados por **MoveIt2** para el control y movimiento del **UR5**, permitiendo la planificación de trayectorias y la ejecución de movimientos del brazo.

---

### **move_ur5**

Este paquete implementa el nodo **`move_ur5`**, que crea el servicio **`activate_arm`** utilizado en los algoritmos de navegación secuencial y paralelo para el control del brazo robótico **UR5**.

---

### **robot_navigation**

Este paquete implementa dos nodos principales: **`navigate_to_goal.py`**, que ejecuta el algoritmo **secuencial**, y **`navigate_to_goal_v2.py`**, que implementa el algoritmo **paralelo**. Para ambos, es necesario ejecutar los siguientes archivos de lanzamiento: **`gazebo_launch.py`**, **`nav2_almacen_launch.py`**, **`move_rviz_launch.py`** (de **ur5_moveit_config**) y **`move_ur5`** para el servicio.

##Adición de modelos en Gazebo

Para la correcta visualización en Gazebo, es necesario añadir a la ruta de Gazebo las carpetas que contienen modelos. Para ello ejecutar en una ventana de comando lso siguientes comandos: 
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/.gazebo/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/ros2_ws/src/husky/husky_description/meshes
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/ros2_ws/src/ur_description/meshes
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/ros2_ws/src/husky_ur5/worlds
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ros2_ws/src/husky_ur5/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/ros2_ws/install/robotiq_description/share
