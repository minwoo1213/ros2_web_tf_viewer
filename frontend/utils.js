// utils.js - PointCloud & Polygon 렌더링 유틸
import * as THREE from './three/three.module.js';

export function renderPointCloud(points, label = 'pointcloud', scene, baseColor = 0x00ff00) {
  const existing = scene.getObjectByName(label);
  if (existing) {
    scene.remove(existing);
    existing.geometry.dispose();
    existing.material.dispose();
  }

  const geometry = new THREE.BufferGeometry();
  const vertices = new Float32Array(points.length * 3);
  const colors = new Float32Array(points.length * 3);

  let minZ = Infinity, maxZ = -Infinity;
  for (const p of points) {
    if (p.z < minZ) minZ = p.z;
    if (p.z > maxZ) maxZ = p.z;
  }
  const zRange = maxZ - minZ || 1;

  points.forEach((p, i) => {
    const idx = i * 3;
    vertices[idx] = p.x;
    vertices[idx + 1] = p.y;
    vertices[idx + 2] = p.z;

    const t = (p.z - minZ) / zRange; // 0~1
    colors[idx] = t;
    colors[idx + 1] = 0.2;
    colors[idx + 2] = 1 - t;
  });

  geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));

  const material = new THREE.PointsMaterial({ size: 0.06, vertexColors: true });
  const pointCloud = new THREE.Points(geometry, material);
  pointCloud.name = label;
  scene.add(pointCloud);
}

export function renderPolygon(points, label = 'polygon', scene, color = 0xff0000) {
  if (!Array.isArray(points) || points.length < 2) return;

  const geometry = new THREE.BufferGeometry();
  const vertices = new Float32Array(points.flatMap(p => [p.x, p.y, p.z]));
  geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));

  const material = new THREE.LineBasicMaterial({ color });
  const line = new THREE.LineLoop(geometry, material);
  line.name = label;

  const existing = scene.getObjectByName(label);
  if (existing) scene.remove(existing);

  scene.add(line);
}

export function showNoObstacleLabel(label = 'obstacle') {
  const el = document.getElementById('no-obstacle');
  if (el) el.style.display = 'block';
}

export function hideNoObstacleLabel(label = 'obstacle') {
  const el = document.getElementById('no-obstacle');
  if (el) el.style.display = 'none';
}

export function parsePointCloud2(msg) {
  const { data, point_step, width, height, fields } = msg;
  if (!data || !point_step || !fields) return [];

  const buf = new DataView(new Uint8Array(data).buffer);
  const xOffset = fields.find(f => f.name === 'x')?.offset;
  const yOffset = fields.find(f => f.name === 'y')?.offset;
  const zOffset = fields.find(f => f.name === 'z')?.offset;

  const points = [];
  for (let i = 0; i < width * height; i++) {
    const base = i * point_step;
    const x = buf.getFloat32(base + xOffset, true);
    const y = buf.getFloat32(base + yOffset, true);
    const z = buf.getFloat32(base + zOffset, true);
    if (isNaN(x) || isNaN(y) || isNaN(z)) continue;
    points.push({ x, y, z });
  }
  return points;
}