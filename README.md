# Litter Bot - Autonomous Litter Collection Robot

ITU Campus (Med Çim) için otonom çöp toplama robotu simülasyonu.

**🎯 Proje Özeti:** Robot devriye gezerken çöp tespit eder, yaklaşır ve toplar (mesafe ≤ 0.3m).

**📊 Detaylı Raporlar:**
- **Türkçe:** [PROJE_RAPORU.md](PROJE_RAPORU.md)
- **English:** [PROJECT_REPORT.md](PROJECT_REPORT.md)

## Sistem Bileşenleri

| Bileşen | Açıklama |
|---------|----------|
| **TurtleBot3 Waffle** | Mobil platform (RGB-D kamera + LiDAR) |
| **Gazebo** | Simülasyon ortamı (30 litter ile arena - 5×6 grid) |
| **Nav2** | Navigation stack (AMCL + DWB controller) |
| **Coverage Planner** | S-pattern alan tarama |
| **Depth Detection** | RGB kamera ile çöp tespiti |
| **Litter Manager** ⭐ | Mesafe kontrolü ve Gazebo entegrasyonu |
| **Coordinator** | State machine - görev yönetimi |

## Kurulum

```bash
# Workspace'e git
cd ~/ws

# Derle
colcon build --symlink-install

# Source
source install/setup.bash
```

## Çalıştırma

### Tek Komutla Full System

```bash
# Her şeyi başlat (Gazebo, SLAM, Nav2, Coverage, Coordinator, RViz)
export TURTLEBOT3_MODEL=waffle
source ~/ws/install/setup.bash
ros2 launch litter_bot_bringup full_system.launch.py
```

### Manuel Başlatma (Ayrı Terminallerde)

**Terminal 1: Gazebo**
```bash
export TURTLEBOT3_MODEL=waffle
source ~/ws/install/setup.bash
ros2 launch litter_bot_gazebo simulation.launch.py
```

**Terminal 2: SLAM**
```bash
source ~/ws/install/setup.bash
ros2 launch litter_bot_navigation slam.launch.py use_sim_time:=true
```

**Terminal 3: Nav2**
```bash
source ~/ws/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
  params_file:=~/ws/install/litter_bot_navigation/share/litter_bot_navigation/config/nav2_params.yaml
```

**Terminal 4: Coverage Planner**
```bash
source ~/ws/install/setup.bash
ros2 run litter_bot_navigation coverage_planner_node --ros-args -p use_sim_time:=true
```

**Terminal 5: Coordinator**
```bash
source ~/ws/install/setup.bash
ros2 run litter_bot coordinator_node --ros-args -p use_sim_time:=true
```

**Terminal 6: RViz** (opsiyonel)
```bash
ros2 run rviz2 rviz2
```

## Görev Başlatma

```bash
# Temizlik görevini başlat
ros2 service call /start_mission std_srvs/srv/Trigger
```

## Test: Manuel Çöp Tespiti

Robot patrol yaparken çöp simüle etmek için:

```bash
# Çöp pozisyonu + tespit sinyali
ros2 topic pub /litter_pose_3d geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}" --once

ros2 topic pub /litter_detected std_msgs/msg/Bool "{data: true}" --once
```

## Koordinatör Davranışı

```
IDLE ──▶ PATROL ──▶ LITTER_DETECTED ──▶ APPROACHING ──▶ PICKING
                          │                                    │
                          │                                    ▼
                          │                              DEPOSITING
                          │                                    │
                          │                                    ▼
                          └────────────────────────────── RETURNING
                                                               │
                                                               ▼
                                                           PATROL
                                                               │
                                                               ▼
                                                         COMPLETED
```

## Servisler

| Servis | Açıklama |
|--------|----------|
| `/start_mission` | Görevi başlat |
| `/stop_mission` | Görevi durdur |
| `/start_coverage` | Coverage başlat |
| `/pause_coverage` | Coverage duraklat |
| `/resume_coverage` | Coverage devam et |

## Topic'ler

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/litter_detected` | Bool | Çöp tespit sinyali |
| `/litter_pose_3d` | PoseStamped | Çöp 3D pozisyonu |
| `/robot_state` | String | Robot durumu |
| `/coverage_active` | Bool | Coverage aktif mi |
| `/cmd_vel` | Twist | Robot hareket komutu |

## Not

- **Kol devre dışı**: OpenMANIPULATOR-X entegrasyonu beklemede
- **YOLO mock**: Gerçek YOLO yerine manuel test sinyalleri kullanılıyor
- **Debounce**: Çöp tespitinden sonra 10 saniye yeni tespit yok sayılıyor
