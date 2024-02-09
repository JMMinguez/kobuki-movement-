# 2024-P3-ForwardTurn

En esta práctica debes crear dos nodo de ROS 2: Uno para avanzar y otro para girar:

1. En nodo de avanzar debe hacer avanzar el robot un metro.
2. El nodo de girar debe hacer gira al robot PI radianes.

Entonces el robot se parará y el nodo finalizará su ejecución.

Debes usar TFs (en partícular `odom`->`base_footprint`) para determinar cuando el robot ha girado o avanzado suficiente.
