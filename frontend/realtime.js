// realtime.js - WebSocket 통해 실시간 PointCloud 수신 및 시각화
import { getScene } from './viewer.js';
import { renderPointCloud } from './utils.js';

export function setupRealtimeWebSocket() {
  const socket = new WebSocket('ws://localhost:8765');
  socket.binaryType = 'arraybuffer';

  socket.onopen = () => console.log('✅ WebSocket connected');
  socket.onerror = err => console.error('❌ WebSocket error', err);

  socket.onmessage = event => {
    const { topic, data } = JSON.parse(event.data);
    const scene = getScene();
    if (!scene || !data) return;
  
    const points = [];
    for (let i = 0; i < data.length; i += 3) {
      points.push({ x: data[i], y: data[i + 1], z: data[i + 2] });
    }
  
    renderPointCloud(points, topic, scene); // label로 topic 사용 가능
  };
}