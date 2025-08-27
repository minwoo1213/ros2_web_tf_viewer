import * as THREE from './three/three.module.js';

export const tfFrames = {};

export function initTfTree(ros) {
  function updateTfTree(transforms, isStatic) {
    transforms.forEach(tf => {
      const parent = tf.header.frame_id.replace(/^\//, '');
      const child = tf.child_frame_id.replace(/^\//, '');
      tfFrames[child] = {
        parent,
        transform: tf.transform,
        isStatic,
        lastUpdated: Date.now()
      };
    });
  }

  new ROSLIB.Topic({ ros, name: '/tf', messageType: 'tf2_msgs/TFMessage' })
    .subscribe(msg => updateTfTree(msg.transforms, false));

  new ROSLIB.Topic({ ros, name: '/tf_static', messageType: 'tf2_msgs/TFMessage' })
    .subscribe(msg => updateTfTree(msg.transforms, true));
}

// 변환 정보 → THREE.Matrix4
export function transformToMatrix(transform) {
  const { translation: t, rotation: q } = transform;
  const matrix = new THREE.Matrix4();
  const quaternion = new THREE.Quaternion(q.x, q.y, q.z, q.w);
  const position = new THREE.Vector3(t.x, t.y, t.z);

  matrix.makeRotationFromQuaternion(quaternion);
  matrix.setPosition(position);
  return matrix;
}

// 특정 frame_id 기준 글로벌 위치 변환 행렬 반환
export function getGlobalTransform(frame_id, tfFrames) {
  let current = frame_id;
  let matrix = new THREE.Matrix4().identity();

  const visited = new Set();
  while (tfFrames[current]) {
    if (visited.has(current)) break; // prevent infinite loop
    visited.add(current);

    const tf = tfFrames[current];
    const local = transformToMatrix(tf.transform);
    matrix = local.multiply(matrix);
    current = tf.parent;
  }

  return matrix;
}