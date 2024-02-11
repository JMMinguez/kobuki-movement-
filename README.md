[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/uJohVe6m)
# 2024-P3-ForwardTurn
## Introducción
El objetivo de la practica es conseguir que el [Kobuki](http://kobuki.yujinrobot.com/about2/) avance exactamente 1m y luego gire pi/2 usando las tf como forma de comprobar la distancia y rotación

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
  
Y para medir la distancia desde el origen así como la rotación del robot he usado la Tf `odom`->`base_footprint`  
```cpp
tf2::Stamped<tf2::Transform> odom2bf;
std::string error;

if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
  auto odom2bf_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);
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
