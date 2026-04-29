#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import binned_statistic_2d
import os
import csv
import argparse
from tf_transformations import euler_from_quaternion

from custom_srvs.srv import Command  

class ErrorAnalysisNode(Node):
    def __init__(self, mode): # filepath, 
        super().__init__('error_analysis_node')
        filepath = '/eurobot_2026/src/camera_cus/error_measurement/error_data.csv'
        self.filepath = os.path.expanduser(filepath)
        # Создаём директорию, если её нет
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        
        self.mode = mode  # 'new' or 'append'

        # Структура данных
        self.data = {
            'x_true': [], 'y_true': [], 'yaw_true': [],
            'x_meas': [], 'y_meas': [], 'yaw_meas': [],
            'dx': [], 'dy': [], 'dyaw': []
        }

        self.last_wheel_odom = None
        self.last_camera_odom = None
        self.wheel_ok = False
        self.camera_ok = False

        # Подписки
        self.wheel_sub = self.create_subscription(Odometry, '/wheel_odom', self.wheel_callback, 10)
        self.camera_sub = self.create_subscription(Odometry, '/camera_odom', self.camera_callback, 10)

        # Сервис
        self.srv = self.create_service(Command, 'odom_analysis', self.command_callback)

        # Загрузка старых данных, если append
        if mode == 'append' and os.path.exists(filepath):
            self.load_from_csv(filepath)
            self.get_logger().info(f'Loaded {len(self.data["x_true"])} previous samples from {filepath}')
        else:
            self.get_logger().info('Starting fresh data collection')

    def wheel_callback(self, msg):
        self.last_wheel_odom = msg
        self.wheel_ok = True

    def camera_callback(self, msg):
        self.last_camera_odom = msg
        self.camera_ok = True

    def get_yaw(self, odom_msg):
        q = odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def sample(self):
        if not (self.wheel_ok and self.camera_ok):
            self.get_logger().warn('No odometry data yet')
            return False

        x_true = self.last_wheel_odom.pose.pose.position.x
        y_true = self.last_wheel_odom.pose.pose.position.y
        yaw_true = self.get_yaw(self.last_wheel_odom)

        x_meas = self.last_camera_odom.pose.pose.position.x
        y_meas = self.last_camera_odom.pose.pose.position.y
        yaw_meas = self.get_yaw(self.last_camera_odom)

        dx = x_meas - x_true
        dy = y_meas - y_true
        dyaw = yaw_meas - yaw_true
        dyaw = (dyaw + np.pi) % (2*np.pi) - np.pi

        self.data['x_true'].append(x_true)
        self.data['y_true'].append(y_true)
        self.data['yaw_true'].append(yaw_true)
        self.data['x_meas'].append(x_meas)
        self.data['y_meas'].append(y_meas)
        self.data['yaw_meas'].append(yaw_meas)
        self.data['dx'].append(dx)
        self.data['dy'].append(dy)
        self.data['dyaw'].append(dyaw)

        self.get_logger().info(f'Sample: pos=({x_true:.2f},{y_true:.2f},yaw={np.rad2deg(yaw_true):.1f}°) '
                               f'error=({dx:.3f},{dy:.3f},{np.rad2deg(dyaw):.1f}°)')
        return True

    def save_to_csv(self):
        # Если append и файл существует – загружаем старые данные
        if self.mode == 'append' and os.path.exists(self.filepath):
            with open(self.filepath, 'r') as f:
                reader = csv.DictReader(f)
                old_data = {key: [] for key in self.data.keys()}
                for row in reader:
                    for key in old_data:
                        old_data[key].append(float(row[key]))
            # Объединяем
            combined = {key: old_data[key] + self.data[key] for key in self.data}
        else:
            combined = self.data

        # Сохраняем объединённые данные
        with open(self.filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=combined.keys())
            writer.writeheader()
            # Транспонируем словарь списков в список строк
            rows = [dict(zip(combined.keys(), vals)) for vals in zip(*combined.values())]
            writer.writerows(rows)

        self.get_logger().info(f'Saved {len(combined["x_true"])} samples to {self.filepath}')

    def load_from_csv(self, filepath):
        with open(filepath, 'r') as f:
            reader = csv.DictReader(f)
            for key in self.data:
                self.data[key] = []
            for row in reader:
                for key in self.data:
                    self.data[key].append(float(row[key]))

    def plot_heatmaps(self):
        if len(self.data['x_true']) == 0:
            self.get_logger().error('No data to plot')
            return

        x_true = np.array(self.data['x_true'])
        y_true = np.array(self.data['y_true'])
        yaw_true = np.array(self.data['yaw_true'])
        dx = np.array(self.data['dx'])
        dy = np.array(self.data['dy'])
        dyaw = np.array(self.data['dyaw'])

        # Разбивка углов на 8 интервалов
        yaw_bins = np.linspace(-np.pi, np.pi, 9)
        # Границы сетки XY
        x_edges = np.linspace(x_true.min(), x_true.max(), 30)
        y_edges = np.linspace(y_true.min(), y_true.max(), 30)

        def plot_one_error(error_vals, title, cmap='coolwarm'):
            fig, axes = plt.subplots(2, 4, figsize=(16, 8), constrained_layout=True)
            axes = axes.flatten()
            for i, (low, high) in enumerate(zip(yaw_bins[:-1], yaw_bins[1:])):
                ax = axes[i]
                mask = (yaw_true >= low) & (yaw_true < high)
                if not np.any(mask):
                    ax.text(0.5, 0.5, f'No data\nyaw [{np.rad2deg(low):.0f}°,{np.rad2deg(high):.0f}°)',
                            transform=ax.transAxes, ha='center')
                    ax.set_xlim(x_edges[0], x_edges[-1])
                    ax.set_ylim(y_edges[0], y_edges[-1])
                else:
                    statistic, _, _, _ = binned_statistic_2d(
                        x_true[mask], y_true[mask], error_vals[mask],
                        statistic='median', bins=[x_edges, y_edges]
                    )
                    masked_stat = np.ma.masked_invalid(statistic)
                    im = ax.pcolormesh(x_edges, y_edges, masked_stat.T, cmap=cmap, shading='auto')
                    fig.colorbar(im, ax=ax, label=title)
                ax.set_title(f'Yaw: {np.rad2deg(low):.0f}–{np.rad2deg(high):.0f}°')
                ax.set_xlabel('X (м)')
                ax.set_ylabel('Y (м)')
                ax.set_aspect('equal')
            fig.suptitle(title, fontsize=16)
            return fig
            

        plot_one_error(dx, 'Ошибка по X (dx), м')
        plot_one_error(dy, 'Ошибка по Y (dy), м')
        plot_one_error(dyaw, 'Ошибка угла (dyaw), рад')
        self.get_logger().info('Displaying heatmaps. Close all windows to continue.')
        plt.show(block=True)  # Блокирует выполнение до закрытия всех окон

    def command_callback(self, request, response):
        cmd = request.command.strip().lower()
        if cmd == 'sample':
            ok = self.sample()
            response.success = ok
            response.message = 'Sample recorded' if ok else 'Failed (no odom)'
        elif cmd == 'save':
            self.save_to_csv()
            response.success = True
            response.message = f'Data saved to {self.filepath}'
        elif cmd == 'plot':
            self.plot_heatmaps()
            response.success = True
            response.message = 'Heatmaps generated'
        elif cmd == 'quit':
            response.success = True
            response.message = 'Quitting node'
            self.get_logger().info('Shutting down...')
            self.destroy_node()
            rclpy.shutdown()
        else:
            response.success = False
            response.message = f'Unknown command: {cmd}'
        return response

def main():
    parser = argparse.ArgumentParser()
    # parser.add_argument('--file', type=str, default='odom_errors.csv', help='CSV file for errors')
    parser.add_argument('--mode', choices=['new', 'append'], default='new', help='Write mode')
    args = parser.parse_args()

    rclpy.init()
    node = ErrorAnalysisNode(args.mode) # args.file, 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()