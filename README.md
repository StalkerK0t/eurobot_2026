# eurobot_2026

Репозиторий Eurobot 2026

## Инструкция по запуску

### Перед запуском контейнера прописать в терминале (вне контейнера):
```
xhost +local:
```

### Внутри контейнера

### Сборка проекта
В директории проекта `/eurobot_2025`:

```
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.sh
```

### Запуск симуляции
```
source install/setup.bash
ros2 launch shesnar launch_sim.launch.py use_sim_time:=True 
```

Управление с клавиатуры:
``` 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```


## Шпаргалка по gz и ros
Список запущенных топиков:
``` 
ros2 topic list  # ROS
gz topic -l      # Gazebo
```

Вывести данные из топика (послушать):
``` 
ros2 topic echo /tf   # ROS
gz topic -e -t /tf    # Gazebo
``` 

Построить граф трансформов:
``` 
ros2 run tf2_tools view_frames
```

Частота публикации сообщений в топик:
``` 
ros2 topic hz /tf
```

Публикация трансформа между фреймами:
``` 
ros2 run tf2_ros tf2_echo map odom
```

Запуск фильтра Калмана вручную
```
ros2 run robot_localization ekf_node --ros-args --params-file src/shesnar/config/ekf_params.yaml
```

Запуск локализации / навигации вручную
```
ros2 launch shesnar localization_launch.py params_file:=src/shesnar/config/nav2_params.yaml map:=src/shesnar/maps/euro_map.yaml use_sim_time:=true

ros2 launch shesnar navigation_launch.py params_file:=src/shesnar/config/nav2_params.yaml map_subscribe_transient_local:=true use_sim_time:=true
```