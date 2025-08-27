import * as THREE from './three/three.module.js';
import { OrbitControls } from './three/examples/jsm/controls/OrbitControls.js';
import { getGlobalTransform, tfFrames } from './tf-tree.js';

let scene, camera, renderer, controls;
let gridHelper;
let autoFollow = true;

export function initRenderer() {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x111111);

  camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.5, 5000);
  camera.position.set(2, 2, 17);  // Foxglove와 비슷한 시작 위치

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  document.body.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;

  controls.addEventListener('start', () => {
    autoFollow = false;
    console.log('🛑 Auto-follow disabled');
  });

  scene.add(new THREE.AxesHelper(1));
  gridHelper = new THREE.GridHelper(10, 10, 0x248eff, 0x248eff); // Foxglove와 동일
  scene.add(gridHelper);

  animate();

  setInterval(() => {
    if (!controls) return;
    const baseTf = getGlobalTransform('base_link', tfFrames);
    const odomTf = getGlobalTransform('odom', tfFrames);
    if (!baseTf || !odomTf) return;

    const basePos = new THREE.Vector3().setFromMatrixPosition(baseTf);
    controls.target.copy(basePos);

    if (autoFollow) {
      const offset = new THREE.Vector3(0, 5, 10);
      camera.position.copy(basePos.clone().add(offset));
      controls.update();
    }

    const odomPos = new THREE.Vector3().setFromMatrixPosition(odomTf);
    const odomQuat = new THREE.Quaternion().setFromRotationMatrix(odomTf);
    gridHelper.position.copy(odomPos);
    gridHelper.quaternion.copy(odomQuat);
  }, 300);
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

export function getScene() {
  return scene;
}

export function getCamera() {
  return camera;
}