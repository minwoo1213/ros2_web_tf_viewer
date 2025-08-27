import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

POINTCLOUD_TOPICS = [
    '/obstacles_msg',
    '/combine_ground_msg',
    '/avoid_path_points_odom',
    '/odom_ldm_fsd_edge_region',
    '/visualize_odom_static_obstacles_occupied_data'
]

class PointCloudWebStreamer(Node):
    def __init__(self):
        super().__init__('pointcloud_web_streamer')
        self.ws_clients = set()  # ✅ 이름 변경 완료

        self.subscriptions = []
        for topic in POINTCLOUD_TOPICS:
            sub = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, t=topic: self.pointcloud_callback(msg, t),
                10
            )
            self.subscriptions.append(sub)

    def pointcloud_callback(self, msg, topic_name):
        points = [
            (x, y, z)
            for x, y, z in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            if abs(x) <= 10.0 and abs(y) <= 10.0
        ]
        if not points:
            return

        payload = {
            "topic": topic_name,
            "data": np.array(points, dtype=np.float32).flatten().tolist()
        }

        message = json.dumps(payload)
        for client in list(self.ws_clients):  # ✅ 수정
            try:
                asyncio.create_task(client.send(message))
            except Exception as e:
                self.get_logger().warn(f"Send error: {e}")

    async def ws_handler(self, websocket, path):
        self.ws_clients.add(websocket)  # ✅ 수정
        try:
            await websocket.wait_closed()
        finally:
            self.ws_clients.remove(websocket)  # ✅ 수정

async def main():
    rclpy.init()
    node = PointCloudWebStreamer()
    server = await websockets.serve(node.ws_handler, "0.0.0.0", 8765)
    print("✅ WebSocket server started on ws://localhost:8765")
    try:
        rclpy.spin(node)
    finally:
        server.close()
        await server.wait_closed()
        rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())