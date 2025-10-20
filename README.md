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