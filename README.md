# ROS2 Web TF Viewer

실시간 ROS 2 토픽(PointCloud2 등)을 **웹 브라우저**에서 시각화하는 경량 뷰어입니다.  
`websocket` 기반 스트리밍으로 프론트엔드(Three.js)와 백엔드를 분리했으며, TF 트리 기반 좌표계를 사용해 여러 소스의 포인트를 전역 좌표계에서 함께 볼 수 있도록 설계되었습니다.

> 테스트 환경: ROS 2 Galactic, Python 3.8+, Chrome/Edge 최신 버전

---

## 주요 기능

- **PointCloud2 실시간 스트리밍** → WebSocket으로 브라우저에 전송
- **여러 토픽 동시 구독** 및 토픽별 데이터 구분
- **좌표계(TF) 기반 시각화**: 전역 좌표에서 포인트 확인 (프론트엔드)
- **경량 프런트엔드**: Three.js + OrbitControls + ES Modules
- **간단한 배포/실행**: 백엔드는 `python` 한 프로세스, 프런트는 정적 호스팅(or 파일 열기)

---

## 디렉터리 구조

```
ros2_web_tf_viewer/
├─ backend/
│  ├─ subscriber.py
│  └─ ws_pointcloud_server.py
├─ frontend/
│  ├─ index.html
│  ├─ app.js
│  ├─ realtime.js
│  ├─ viewer.js
│  ├─ tf-tree.js
│  ├─ utils.js
│  └─ three/         # Three.js (ESM)
└─ .git/
```

---

## 시스템 구성

```
[ROS 2 Nodes] ──(PointCloud2 등)──> [Backend]
                                     └─ 구독 후 필터링/직렬화
                                     └─ WebSocket(ws://0.0.0.0:8765) 방송
                                              ↓
                                       [Frontend (Three.js)]
                                       └─ 토픽별 포인트 버퍼링 & 렌더링
                                       └─ TF Tree/OrbitControls
```

- 백엔드: ROS 2에서 `PointCloud2`를 구독하고 `(x,y,z)` 포인트를 평탄화하여 JSON으로 브로드캐스트
- 프런트엔드: WebSocket으로 수신하여 토픽별로 색/레이어 분리 렌더링(구현 예시는 `frontend/*.js` 참고)

---

## 요구 사항

### 공통
- Python 3.8+
- ROS 2 Galactic (또는 호환 버전) 설치 및 `setup.bash` 소스
- 브라우저: Chrome/Edge 최신권장

### Python 의존성
```bash
pip install websockets numpy sensor_msgs-py rclpy
# (선택) rosbridge_library를 활용하는 경우:
# pip install rosbridge-library
```

> Ubuntu에서 ROS 2 Python 패키지는 배포판 패키지/오버레이에 따라 `apt` 또는 rosdep로 설치되어 있을 수 있습니다.

---

## 설치 & 실행

### 1) ROS 2 환경 설정
```bash
source /opt/ros/galactic/setup.bash
# 워크스페이스 사용 시:
# source ~/ros2_ws/install/setup.bash
```

### 2) 백엔드(WebSocket 서버) 실행
```bash
cd backend
python3 ws_pointcloud_server.py
# 출력 예) "✅ WebSocket server started on ws://localhost:8765"
```

- 기본 포트: `8765`
- 기본 구독 토픽: 
  - `/obstacles_msg`
  - `/combine_ground_msg`
  - `/avoid_path_points_odom`
  - `/odom_ldm_fsd_edge_region`
  - `/visualize_odom_static_obstacles_occupied_data`

> 필요 시 `POINTCLOUD_TOPICS` 리스트를 수정하세요.  
> XY 평면에서 `|x|<=10.0`, `|y|<=10.0` 범위 외 포인트는 필터링됩니다.

### 3) 프런트엔드 실행
옵션 A — 간편 실행(정적 서버):
```bash
cd frontend
python3 -m http.server 8080
# 브라우저에서 http://localhost:8080 열기
```

옵션 B — 파일 직접 열기:
- `frontend/index.html`을 브라우저로 직접 열기(Live Server 확장도 OK)

프런트엔드에서 기본 WebSocket 주소는 `ws://localhost:8765`를 사용합니다(필요 시 스크립트 내 주소 변경).

---

## 메시지 포맷(WebSocket)

백엔드는 토픽별로 아래와 같은 JSON을 브로드캐스트합니다.

```json
{
  "topic": "/odom_ldm_fsd_edge_region",
  "data": [x1, y1, z1, x2, y2, z2, ...]  // float32 1D 배열(플랫)
}
```

- `data`는 `(x, y, z)` 삼중항이 순차적으로 들어있는 **flattened float 배열**입니다.
- 각 메시지는 특정 `topic`의 포인트 클라우드를 나타냅니다.
- 정밀도/크기를 고려해 float32로 직렬화되어 전송됩니다.

---

## 구성(커스터마이즈)

### 1) 구독 토픽 변경
`backend/ws_pointcloud_server.py`:
```python
POINTCLOUD_TOPICS = [
    '/obstacles_msg',
    '/combine_ground_msg',
    '/avoid_path_points_odom',
    '/odom_ldm_fsd_edge_region',
    '/visualize_odom_static_obstacles_occupied_data'
]
```

### 2) 필터 범위 변경
XY 필터는 콜백 내부에서 적용됩니다.
```python
if abs(x) <= 10.0 and abs(y) <= 10.0:  # 필요 시 값 조정
```

### 3) 포트 변경
`websockets.serve(node.ws_handler, "0.0.0.0", 8765)`에서 `8765` 수정

---

## QoS/구독 유틸(선택)

`backend/subscriber.py`에는 ROS 2 메시지를 dict로 변환하는 보조 구독자가 포함됩니다.  
QoS는 `BEST_EFFORT`, `VOLATILE`, `depth=10`으로 설정되어 저지연 수신에 적합합니다.

```python
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10
)
```

- 내부에서 `rosbridge_library.internal.message_conversion.extract_values`를 이용해 메시지를 dict로 변환합니다.
- 현재 기본 스트리머(`ws_pointcloud_server.py`)는 `sensor_msgs_py.point_cloud2.read_points`로 직접 파싱합니다.

---

## 트러블슈팅

- **브라우저가 포인트가 안 보임**  
  - 실제 ROS 토픽이 발행 중인지 `ros2 topic echo /<topic>`로 확인
  - WebSocket 연결 주소(`ws://host:port`) 확인
  - 포인트가 XY 필터 범위 밖인지 확인(필터 값 조정)

- **성능 튜닝**  
  - 필터링 범위 축소로 전송 데이터 감소
  - 토픽 수 줄이기/다운샘플링 적용
  - 프런트엔드에서 프레임 단위로 포인트 버퍼링/씬 갱신 최적화

- **QoS 불일치(ROS 2)**  
  - 퍼블리셔 QoS가 `BEST_EFFORT`가 아닐 경우 수신 누락 가능 → 퍼블리셔/서브스크라이버 QoS를 조정

---

## 라이선스
사내/프로젝트 정책을 따릅니다. 필요 시 라이선스를 명시하세요.

---

## 변경 이력
- 2025-08-27: 초기 README 작성
