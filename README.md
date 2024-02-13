[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/uJohVe6m)
# 2024-P3-ForwardTurn
## Introducción
El objetivo de la practica es conseguir que el [Kobuki](https://robots.ros.org/kobuki/) avance exactamente 1m y luego gire pi/2 usando las tf como forma de comprobar la distancia y rotación

## Descrioción y procedimiento
En esta práctica, he utilizado como bsae el paquete [**ASR_2024**](https://github.com/Docencia-fmrico/ASR_2024) proporcionado por [fmrico](https://github.com/fmrico).  

El funcionamiento principal del robot viene dado por una FSM (Finite States Machine) que va cambiando de entre 3 posibles estados: FORWARD, TURN y STOP.
```cpp
switch (state_) {
    case FORWARD:
      RCLCPP_INFO(get_logger(), "Moving forward!");
      ...
      break;

    case TURN:
      RCLCPP_INFO(get_logger(), "Rotating!");
      ...
      break;

    case STOP:
      RCLCPP_INFO(get_logger(), "STOP!");
      ....
      break;
  }
```
![Diagrama_FSM](https://github.com/Docencia-fmrico/p3-forwardturn-jmartinm2021/assets/92941332/10f33708-1f11-4492-abcd-7083a98e1fa9)
  
Y para medir la distancia desde el origen así como la rotación del robot he usado la Tf `odom`->`base_footprint`, este metodo solo funciona si ejectuamos el paquete una vez ya que si no habria que apagar y encender el robot de nuevo para reiniciar odom
```cpp
tf2::Stamped<tf2::Transform> odom2bf;
std::string error;

if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
  auto odom2bf_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);
```

Sin embargo podemos guardar la posicion inicial cada vez que ejecutamos el programa y calcular la posicion en función esa posición inicial y el 'base_footprint' actual de la siguiente forma: bf2bf1 = bf2odom * odom2bf1
```cpp
tf2::Stamped<tf2::Transform> odom2bf;
  std::string error;
  //  Gets the tf from 'odom' to 'base_footprint' at the start position
  if (start_)
  {
    
    if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
    auto odom2bf_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);
      tf2::fromMsg(odom2bf_msg, odom2bf);
    }
    start_ = !start_;
  }

  tf2::Transform odom2bf_inverse = odom2bf.inverse();
  tf2::Stamped<tf2::Transform> odom2bf1;
  
  //  Gets the tf between 'odom' and actual 'base_footprint'
  if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
    auto odom2bf1_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);

    tf2::fromMsg(odom2bf1_msg, odom2bf1);

    // Gets the tf from start 'base_footprint' and actual 'base_footprint'
    tf2::Transform bf2bf1 = odom2bf_inverse * odom2bf1;
```
## Launcher y ejecucuión
Como en esta práctica estabamos tratando con TFs he creido necesario implementar en el launcher lo siguiente:
```python
def generate_launch_description():
    forward_turn_cpp_dir = get_package_share_directory('forward_turn_cpp')

    rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        arguments=['-d', [os.path.join(forward_turn_cpp_dir, 'config', 'forward.rviz')]],
        )

        ld = LaunchDescription()
        ld.add_action(rviz2_cmd)

        return ld
```
Así se abrirá el rviz y podremos estar viendo en todo momento las TFs.  

Para ejecutar nuestro paquete debemos poner el la terminal de Linux
```shell
ros2 launch forward_turn_cpp launch.py
```
## Videos demostración
En este primer video se ve el funcionamiento de la maquina de estados, sin embargo aun no hace el TURN de manera correcta:

[Prueba1.webm](https://github.com/Docencia-fmrico/p3-forwardturn-jmartinm2021/assets/92941332/ea76c794-bc62-4223-8cbd-aef447d0f0dd)
  
## Enunciado

En esta práctica debes crear dos nodo de ROS 2: Uno para avanzar y otro para girar:

1. En nodo de avanzar debe hacer avanzar el robot un metro.
2. El nodo de girar debe hacer gira al robot PI radianes.

Entonces el robot se parará y el nodo finalizará su ejecución.

Debes usar TFs (en partícular `odom`->`base_footprint`) para determinar cuando el robot ha girado o avanzado suficiente.
