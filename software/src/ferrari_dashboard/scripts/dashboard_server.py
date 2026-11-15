#!/usr/bin/env python3

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from ferrari_control.msg import TrackingError


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard_node")
        self.port = self.declare_parameter("port", 8080).value
        self.host = self.declare_parameter("host", "127.0.0.1").value
        self.camera_topic = self.declare_parameter("camera_topic", "/sensing/camera/image_raw/compressed").value

        self.lock = threading.Lock()
        self.state = {
            "pose": None,
            "speed": 0.0,
            "yaw_rate": 0.0,
            "target_speed": None,
            "cmd_speed": 0.0,
            "cmd_steer": 0.0,
            "lateral_error": None,
            "heading_error": None,
            "mode": None,
            "path": [],
            "scan": [],
            "goal": None,
            "map": None,
            "message": "",
        }
        self.map_png = None
        self.map_version = 0
        self.camera_frame = None
        self.camera_stamp = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, "/localization/odom", self.on_odom, 10)
        self.create_subscription(TrackingError, "/control/tracking_error", self.on_tracking_error, 10)
        self.create_subscription(String, "/control/mode", self.on_mode, latched)
        self.create_subscription(OccupancyGrid, "/mapping/map", self.on_map, latched)
        self.create_subscription(Path, "/planning/global_path", self.on_path, latched)
        self.create_subscription(LaserScan, "/sensing/lidar/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, self.camera_topic, self.on_camera, qos_profile_sensor_data)
        self.create_subscription(AckermannDriveStamped, "/vehicle/ackermann_cmd", self.on_command, 10)

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.switch_client = self.create_client(Trigger, "/control/switch_controller")
        self.arm_client = self.create_client(Trigger, "/vehicle/toggle_actuators")

        self.create_timer(0.1, self.update_pose)
        self.latest_scan = None

    def on_odom(self, msg):
        with self.lock:
            self.state["speed"] = msg.twist.twist.linear.x
            self.state["yaw_rate"] = msg.twist.twist.angular.z

    def on_tracking_error(self, msg):
        with self.lock:
            self.state["target_speed"] = msg.target_speed
            self.state["lateral_error"] = msg.lateral_error
            self.state["heading_error"] = msg.heading_error

    def on_command(self, msg):
        with self.lock:
            self.state["cmd_speed"] = msg.drive.speed
            self.state["cmd_steer"] = msg.drive.steering_angle

    def on_mode(self, msg):
        with self.lock:
            self.state["mode"] = msg.data

    def on_scan(self, msg):
        self.latest_scan = msg

    def on_camera(self, msg):
        self.camera_frame = bytes(msg.data)
        self.camera_stamp = time.time()

    def on_path(self, msg):
        step = max(1, len(msg.poses) // 300)
        points = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses[::step]]
        with self.lock:
            self.state["path"] = points

    def on_map(self, msg):
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        image = np.full(data.shape, 128, dtype=np.uint8)
        image[(data >= 0) & (data < 65)] = 235
        image[data >= 65] = 25
        ok, encoded = cv2.imencode(".png", cv2.flip(image, 0))
        if not ok:
            return
        with self.lock:
            self.map_png = encoded.tobytes()
            self.map_version += 1
            self.state["map"] = {
                "version": self.map_version,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                "resolution": msg.info.resolution,
                "width": msg.info.width,
                "height": msg.info.height,
            }

    def lookup(self, target, source):
        try:
            t = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            return t.transform
        except Exception:
            return None

    def update_pose(self):
        transform = self.lookup("map", "base_link")
        frame = "map"
        if transform is None:
            transform = self.lookup("odom", "base_link")
            frame = "odom"
        pose = None
        if transform is not None:
            pose = {"x": transform.translation.x, "y": transform.translation.y, "yaw": yaw_of(transform.rotation), "frame": frame}

        scan_points = []
        scan = self.latest_scan
        laser = self.lookup(frame, "lidar_link") if scan is not None and transform is not None else None
        if scan is not None and laser is not None:
            lyaw = yaw_of(laser.rotation)
            c, s = math.cos(lyaw), math.sin(lyaw)
            for i in range(0, len(scan.ranges), 2):
                r = scan.ranges[i]
                if math.isfinite(r) and scan.range_min < r < scan.range_max:
                    a = scan.angle_min + i * scan.angle_increment
                    px, py = r * math.cos(a), r * math.sin(a)
                    scan_points.append([round(laser.translation.x + c * px - s * py, 2), round(laser.translation.y + s * px + c * py, 2)])

        with self.lock:
            self.state["pose"] = pose
            self.state["scan"] = scan_points

    def snapshot(self):
        with self.lock:
            data = dict(self.state)
        data["camera"] = (time.time() - self.camera_stamp) < 2.0
        data["time"] = time.time()
        return data

    def publish_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(msg)
        with self.lock:
            self.state["goal"] = {"x": float(x), "y": float(y), "yaw": float(yaw)}
            self.state["message"] = "goal sent"

    def call_trigger(self, client, label):
        if not client.service_is_ready():
            with self.lock:
                self.state["message"] = f"{label}: service not available"
            return

        def done(future):
            try:
                result = future.result()
                text = result.message if result.message else ("ok" if result.success else "failed")
            except Exception as error:
                text = str(error)
            with self.lock:
                self.state["message"] = f"{label}: {text}"

        client.call_async(Trigger.Request()).add_done_callback(done)


def make_handler(node, page):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send_bytes(self, body, content_type, status=200):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self.send_bytes(page, "text/html; charset=utf-8")
            elif path == "/api/state":
                self.send_bytes(json.dumps(node.snapshot()).encode(), "application/json")
            elif path == "/map.png":
                with node.lock:
                    png = node.map_png
                if png is None:
                    self.send_bytes(b"", "image/png", 404)
                else:
                    self.send_bytes(png, "image/png")
            elif path == "/camera.mjpg":
                self.stream_camera()
            else:
                self.send_bytes(b"not found", "text/plain", 404)

        def stream_camera(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = 0.0
            try:
                while rclpy.ok():
                    frame, stamp = node.camera_frame, node.camera_stamp
                    if frame is not None and stamp != last:
                        last = stamp
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
                    time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length) or b"{}")
            if self.path == "/api/goal":
                node.publish_goal(body["x"], body["y"], body.get("yaw", 0.0))
            elif self.path == "/api/switch_controller":
                node.call_trigger(node.switch_client, "controller")
            elif self.path == "/api/toggle_actuators":
                node.call_trigger(node.arm_client, "actuators")
            else:
                self.send_bytes(b"not found", "text/plain", 404)
                return
            self.send_bytes(b"{}", "application/json")

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    page_path = os.path.join(get_package_share_directory("ferrari_dashboard"), "public", "index.html")
    with open(page_path, "rb") as f:
        page = f.read()

    server = ThreadingHTTPServer((node.host, node.port), make_handler(node, page))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"Ferrari dashboard on http://{node.host}:{node.port}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
