import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import sys


class ColorCamera(Node):
    def __init__(self):
        super().__init__('color_camera')
        self.get_logger().info(f"OpenCV version: {cv2.__version__}")
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.color_pub = self.create_publisher(String, 'color_pattern', 10)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image', 
            self.callback,
            10)

        self.bridge = CvBridge()
        
        # Цветовые диапазоны в HSV
        self.blue_lower = np.array([100, 50, 50])
        self.blue_upper = np.array([130, 255, 255])
        self.orange_lower = np.array([5, 100, 100])
        self.orange_upper = np.array([15, 255, 255])
        
        self.get_logger().info('Color Detector Node started')

    def find_color_bounds(self, image):
        """Находит левую и правую границы по цветам"""
        h, w = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Объединяем синий и оранжевый в одну маску
        mask_blue = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        mask_orange = cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        mask = cv2.bitwise_or(mask_blue, mask_orange)
        
        # Ищем левую границу (сверху и снизу)
        left_top = w
        left_bottom = w
        right_top = 0
        right_bottom = 0
        
        # Верхняя половина (первые 40% высоты)
        for y in range(int(h * 0.1), int(h * 0.4)):
            for x in range(w):
                if mask[y, x] > 0:
                    if x < left_top:
                        left_top = x
                    break
        
        # Нижняя половина (последние 40% высоты)
        for y in range(int(h * 0.6), int(h * 0.9)):
            for x in range(w):
                if mask[y, x] > 0:
                    if x < left_bottom:
                        left_bottom = x
                    break
        
        # Ищем правую границу (сверху и снизу)
        for y in range(int(h * 0.1), int(h * 0.4)):
            for x in range(w - 1, -1, -1):
                if mask[y, x] > 0:
                    if x > right_top:
                        right_top = x
                    break
        
        for y in range(int(h * 0.6), int(h * 0.9)):
            for x in range(w - 1, -1, -1):
                if mask[y, x] > 0:
                    if x > right_bottom:
                        right_bottom = x
                    break
        
        # Если не нашли, используем края изображения
        if left_top == w:
            left_top = int(w * 0.1)
        if left_bottom == w:
            left_bottom = int(w * 0.1)
        if right_top == 0:
            right_top = int(w * 0.9)
        if right_bottom == 0:
            right_bottom = int(w * 0.9)
        
        # Четыре угла трапеции
        corners = np.array([
            [left_top, int(h * 0.2)],      # левый верхний
            [right_top, int(h * 0.2)],     # правый верхний
            [right_bottom, int(h * 0.8)],  # правый нижний
            [left_bottom, int(h * 0.8)]    # левый нижний
        ], dtype=np.float32)
        
        self.get_logger().info(f"Bounds - left_top:{left_top}, left_bottom:{left_bottom}, right_top:{right_top}, right_bottom:{right_bottom}")
        
        return corners

    def split_into_zones(self, image, corners):
        """Делит трапецию на 4 равные зоны слева направо"""
        h, w = image.shape[:2]
        
        # Интерполируем границы для каждого x
        left_line = np.linspace(corners[0][1], corners[3][1], w)
        right_line = np.linspace(corners[1][1], corners[2][1], w)
        
        zone_width = w // 4
        colors = []
        
        for i in range(4):
            x_start = i * zone_width
            x_end = (i + 1) * zone_width
            x_center = (x_start + x_end) // 2
            
            # Определяем вертикальные границы зоны
            y_top = int((left_line[x_center] + right_line[x_center]) / 2)
            y_bottom = int((left_line[x_center] + right_line[x_center] + 100) / 2)
            
            # Уточняем границы
            y_top = max(0, y_top - 20)
            y_bottom = min(h, y_bottom + 20)
            
            # Вырезаем зону
            zone = image[y_top:y_bottom, x_start:x_end]
            
            if zone.size > 0:
                color = self.get_zone_color(zone)
                colors.append(color)
                self.get_logger().info(f"Zone {i}: color={color}")
            else:
                colors.append(-1)
        
        return colors

    def get_zone_color(self, zone):
        """Определяет цвет в зоне"""
        hsv = cv2.cvtColor(zone, cv2.COLOR_BGR2HSV)
        
        mask_blue = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        mask_orange = cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        
        blue_pixels = cv2.countNonZero(mask_blue)
        orange_pixels = cv2.countNonZero(mask_orange)
        
        if blue_pixels > orange_pixels and blue_pixels > 100:
            return 0
        elif orange_pixels > blue_pixels and orange_pixels > 100:
            return 1
        return -1

    def publish_color_pattern(self, colors):
        """Публикует паттерн цветов"""
        pattern_str = ' '.join(str(c) for c in colors)
        msg = String()
        msg.data = pattern_str
        self.color_pub.publish(msg)
        self.get_logger().info(f"Published: {pattern_str}")

    def callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return
        
        # Находим границы по цветам
        corners = self.find_color_bounds(image)
        
        # Делим на 4 зоны и определяем цвета
        colors = self.split_into_zones(image, corners)
        
        if all(c != -1 for c in colors):
            self.publish_color_pattern(colors)
        
        # Визуализация
        display = image.copy()
        
        # Рисуем трапецию
        pts = corners.astype(int)
        cv2.polylines(display, [pts], True, (0, 255, 0), 2)
        
        # Рисуем зоны
        h, w = display.shape[:2]
        zone_width = w // 4
        
        for i in range(4):
            x = i * zone_width + zone_width // 2
            cv2.line(display, (x, 0), (x, h), (255, 0, 0), 1)
            
            if i < len(colors):
                cv2.putText(display, str(colors[i]), (x - 20, h // 2), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("Camera", display)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise SystemExit

    def test_with_image(self, image_path):
        """Тестирование на одном изображении"""
        image = cv2.imread(image_path)
        if image is None:
            self.get_logger().error(f"Can't load image: {image_path}")
            return
        
        self.get_logger().info(f"Testing on: {image_path}")
        
        # Находим границы
        corners = self.find_color_bounds(image)
        
        # Делим на зоны
        colors = self.split_into_zones(image, corners)
        
        self.get_logger().info(f"Colors: {colors}")
        self.publish_color_pattern(colors)
        
        # Визуализация
        display = image.copy()
        
        # Рисуем трапецию
        pts = corners.astype(int)
        cv2.polylines(display, [pts], True, (0, 255, 0), 2)
        
        # Рисуем зоны
        h, w = display.shape[:2]
        zone_width = w // 4
        
        for i in range(4):
            x = i * zone_width + zone_width // 2
            cv2.line(display, (x, 0), (x, h), (255, 0, 0), 1)
            
            if i < len(colors):
                cv2.putText(display, str(colors[i]), (x - 20, h // 2), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("Result", display)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = ColorCamera()
    
    import sys
    if len(sys.argv) > 1:
        node.test_with_image(sys.argv[1])
    else:
        try:
            rclpy.spin(node)
        except SystemExit:
            pass
    
    node.destroy_node()
    sys.exit(0)


if __name__ == '__main__':
    main()