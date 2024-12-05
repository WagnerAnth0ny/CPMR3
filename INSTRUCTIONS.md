# Intruçôes sobre como rodar o projeto

Primeiramente é preciso buildar o projeto usando:

```
colcon build
```

Após isso rode o script para fazer a source do workspace

```
source CPMR3/ros2_ws/src/install/setup.bash
```

## Problema 2.1

Vamos começar rodando o comando:

```
ros2 launch cpmr_ch2 gazebo.launch.py
```

Agora vamos popular a simulação usando o comando:

```
ros2 launch cpmr_ch2 build_map.launch.py
```

Por fim, vamos mover o robô usando o comando:

```
ros2 run cpmr_ch2 drive_to_goal --ros-args -p _goal_x:=1.0 -p _goal_y:=2.0 -p _goal_t:=1.5
```

## Problema 2.2

O primeiro comando que precisamos rodar é: 

```
ros2 launch cpmr_ch2 gazebo.launch.py
```

Agora vamos popular a simulação usando o comando:

```
ros2 launch cpmr_ch2 build_map.launch.py
```

Por fim, vamos rodar o comando:

```
ros2 run cpmr_ch2 bug_two --ros-args -p _goal_x:=4.0 -p _goal_y:=5.0 -p _goal_t:=1.5
```

