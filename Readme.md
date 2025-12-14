# Robot 6dof - Integracion con ROS2  
Creado por: Kenyi reyes De Lama 

## Requisitos previos
- Windows 11 con WSL2 y distribución Ubuntu 22.04
- ROS 2 Humble instalado dentro de WSL
- VS Code con extensiones Remote-WSL
- Conexión física al robot y al ESP32 (previamente configurado con el MicroROS Agent)
- Fuente de alimentación externa con 6v limitada a 3A 

> [Nota: si se esta usando WSL, se debe de instalar usbipd en WSL2, para poder conectar el dispositivo USB del ESP32 a WSL2 usando usbip.]

## Paso 1: Programar en microsROS el ESP32
- Con un esp32 configurado con el MicroROS Agent, programar el ESP32 con los codigos INO.

## Paso 2: Conectar el robot a WSL
Exportar el puerto USB del ESP32 a WSL2 usando usbipd.
Esto abrirlo como adminitsrator en powershell, y ejecutar los siguientes comandos:
```bash
usbipd list
```
```bash
usbipd attach --busid 1-3 --wsl Ubuntu-22.04
```

## Paso 3: Activar micro-ROS
1. ```bash
    cd ws_microros/ 
    ```
2. ```bash
    colcon build 
    ```
3. ```bash
    source install/setup.bash
    ```
4. ```bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
    ```
## Paso 4: Activar los nodos
1. ```bash 
    cd ws_robot/ 
    ```
2. ```bash
    colcon build
    ```
3. ```bash
    source install/setup.
    ```
Lanzamos  los nodos:

```bash
ros2 launch lab1 bringup_sliders_servos_sim.launch.py
```
```bash
ros2 launch lab1 nodo_hola.py
```

