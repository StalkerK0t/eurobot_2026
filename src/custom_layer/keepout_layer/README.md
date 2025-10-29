# Keepout layer

Координаты зон задаются в `eurobot_2026/src/shesnar/config/nav2_params.yaml` в формате [X, Y, lengthX, lengthY].

```
        keepout_zone_array: [0.375, 1.775, 0.45, 0.45,  # A
                             0.225, 0.875, 0.45, 0.45,  # B
                             0.225, 0.075, 0.45, 0.15,  # C
                             0.775, 0.075, 0.45, 0.15,  # D
                             1.225, 0.225, 0.45, 0.45,  # E
                             #----------------------------#
                             1.775, 0.225, 0.45, 0.45,  # F
                             2.225, 0.075, 0.45, 0.15,  # G
                             2.775, 0.075, 0.45, 0.15,  # H
                             2.775, 0.875, 0.45, 0.45,  # I
                             2.625, 1.775, 0.45, 0.45,] # J 
``` 

![keepout layer]([docs/keepout.png](https://github.com/StalkerK0t/eurobot_2026/blob/nonepenguin/src/custom_layer/keepout_layer/docs/table_FINALE_1.0_keepout.png))

Возможные проблемы: система координат на реальном поле и в симуляции должны совпадать (или придётся изменять эти значения).

Для отключения зон нужно отправлять их названия (последовательные латинские буквы - Message Type: std_msgs/msg/String) в топик `keepout_zone`.

Через терминал:
```
ros2 topic pub /keepout_zone map_points std_msgs/msg/String "{data: "B"}"
```

Через планировщик маршрута :
```
self.publisher_ = self.create_publisher(String, '/keepout_zone', 10)
msg = String()
msg.data = "B"  
self.publisher_.publish(msg)
```
