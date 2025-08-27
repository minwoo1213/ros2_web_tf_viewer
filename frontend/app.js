import { initRenderer } from './viewer.js';
import { initTfTree } from './tf-tree.js';
import { setupRealtimeWebSocket } from './realtime.js';
import { renderPointCloud, renderPolygon } from './utils.js';

const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
ros.on('connection', () => console.log('✅ Connected to rosbridge'));
ros.on('error', err => console.error('❌ Connection error:', err));

initTfTree(ros);
initRenderer();
setupRealtimeWebSocket();

let scene, camera, renderer, controls, gridHelper;
let autoFollow = true;

const topics = [
  {
    name: '/collision_dagerous_roi',
    type: 'geometry_msgs/PolygonStamped',
    color: 0xff0000,
    render: renderPolygon,
    label: 'AEB ROI',
  },
  {
    name: '/newbie_shape_polygon_roi',
    type: 'geometry_msgs/PolygonStamped',
    color: 0xff8800,
    render: renderPolygon,
    label: '뉴비 ROI',
  },
  {
    name: '/combine_ground_msg',
    type: 'sensor_msgs/PointCloud2',
    color: 0xffffff,
    render: renderPointCloud,
    label: '지면 3D',
  },
  {
    name: '/obstacles_msg',
    type: 'sensor_msgs/PointCloud2',
    color: 0x00c3ff,
    render: renderPointCloud,
    label: '장애물',
  },
  {
    name: '/odom_ldm_fsd_edge_region',
    type: 'sensor_msgs/PointCloud2',
    color: 0xff0000,
    render: renderPointCloud,
    label: '진입불가 영역',
  },
  {
    name: '/avoid_path_points_odom',
    type: 'sensor_msgs/PointCloud2',
    color: 0x00ff0d,
    render: renderPointCloud,
    label: '회피 경로',
  },
  {
    name: '/visualize_odom_static_obstacles_occupied_data',
    type: 'sensor_msgs/PointCloud2',
    color: 0xff2222,
    render: renderPointCloud,
    label: '스태틱 장애물',
  }
];

topics.forEach(({ name, type, color, render, label }) => {
  const topic = new ROSLIB.Topic({ ros, name, messageType: type });

  topic.subscribe(msg => {
    const frame_id = msg.header.frame_id.replace(/^\//, '');
    const transform = getGlobalTransform(frame_id, tfFrames);

    let points = [];

    if (type === 'geometry_msgs/PolygonStamped') {
      if (!msg.polygon?.points?.length) {
        showNoObstacleLabel(label);
        return;
      }
      points = msg.polygon.points.map(p => transformPoint(p, transform));
    } else if (type === 'sensor_msgs/PointCloud2') {
      points = parsePointCloud2(msg);
      if (!points.length) {
        showNoObstacleLabel(label);
        return;
      }
      points = points.map(p => transformPoint(p, transform));
    }

    hideNoObstacleLabel(label);
    render(points, label, getScene(), color);
  });
});

function transformPoint(p, matrix) {
  const vec = new THREE.Vector3(p.x, p.y, p.z);
  vec.applyMatrix4(matrix);
  return { x: vec.x, y: vec.y, z: vec.z };
}