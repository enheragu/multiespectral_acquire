# Multiespectral Acquire

Generic metapackage for **synchronized multi-sensor data acquisition** on mobile robots. The drivers, buffer synchronization pipeline, and storage system are configurable via ROS 2 parameters and launch files — any combination of cameras and sensors can be assembled.

| Package | Description |
|---------|-------------|
| [multiespectral_acquire](multiespectral_acquire/) | Camera drivers (Basler, FLIR), timestamp synchronization, buffer/storage pipeline, FOV crop |
| [multiespectral_acquire_gui](multiespectral_acquire_gui/) | Generic Flask + SocketIO web dashboard — start/stop recording, live camera feeds, frame-rate monitoring |
| [temperature_driver](temperature_driver/) | DHT22 temperature/humidity sensor via NodeMCU (ESP8266) |

---

## HITOS deployment

This package runs on the **HITOS multiespectral payload** (Raspberry Pi 5 OBC). For hardware details — network topology, power budget, PoE wiring, PTP setup, and systemd services — see [`hitos_setup/README.md`](../hitos_setup/README.md).

Key HITOS-specific parameters:

| Parameter | Value | Reason |
|-----------|-------|--------|
| `visible_use_ptp` | `false` | Basler acA1600-60gc has no PTP; uses software `TimestampCalibration` |
| `lwir_use_ptp` | `true` | **Lock-aware**: the FLIR adapter enables PTP but verifies a real `ptpServoStatus = Locked` + UTC sanity, and falls back to software calibration if not. On this rig PTP never locks (the master is fine, the slaves don't — see `hitos_setup` → Timestamp synchronization), so the FLIR effectively runs on software calibration. |
| `dataset_output_path` | `/media/arvc/DATASETS/images_eeha` | External HDD (set by `hitos_sync.service`, the disk writer) |

**Launch files:** `multiespectral_launch.py` runs everything in one process tree
(manual use). For the systemd deployment the pipeline is split:
`cameras_only.launch.py` (Basler + FLIR, `hitos_cameras.service`) and
`capture_sync.launch.py` (lidar crop + buffer compositor,
`hitos_sync.service`). Both take the same `session_folder`; the services share
it via `/tmp/hitos_session.env`.

**Buffer compositor performance notes (2026-06):** high-rate handlers subscribe
with `raw=True` (serialized CDR bytes, parsed timestamp at a fixed offset) and
deserialize only the matched message at trigger rate; disk writes go through a
small thread pool; the process uses rclpy's `EventsExecutor`; handler nodes
disable per-node parameter services/rosout. All of this together removes the
per-message Python deserialization of the 30 Hz LWIR and 10 Hz Ouster streams
and the wait-set churn of the old MultiThreadedExecutor.

**Generic timestamp handling in `buffer_handler_node.py`** — each handler aligns its
source onto the trigger's wall-clock timescale via per-handler params (all sensor-agnostic;
the compositor just forwards them):
- `use_raw` — buffer serialized CDR bytes, read `header.stamp` at a fixed offset; deserialize
  only the matched message (~1 Hz). Avoids per-message Python deserialization.
- `stamp_from_arrival` — buffer by message arrival time instead of `header.stamp`
  (fallback for a sensor whose own stamp is unusable).
- `clock_offset_topic` — subscribe to a `std_msgs/Float64` offset (seconds) and add it to
  every stamp (buffering + republished). Used to map a free-running sensor clock to
  wall-clock. The Ouster runs `TIME_FROM_INTERNAL_OSC` (PTP won't lock on this rig) and
  `hitos_setup/ouster_recal_node` publishes `/ouster/clock_offset` (calibrated from the
  100 Hz IMU). See `hitos_setup/README.md` → "Timestamp synchronization".

**LiDAR point cloud:** the Ouster driver runs `organized: false` → a **dense** cloud
(~0.85 MB, valid points only) instead of the full 1024×128 grid (6.29 MB, 87 % zeros
outside the azimuth window). This lets the Python compositor receive the cloud at the full
10 Hz (the 6 MB organized cloud capped Python at ~5 Hz, degrading sync). Consequently
`pointcloud_crop_node` filters by the per-point **`ring` field** (beam index → elevation,
`sensor_v_fov_deg` param) rather than slicing a row/col rectangle of the (now absent) grid.

---

## Architecture — HITOS Multiespectral (ROS 2)

```mermaid
flowchart LR
    subgraph HW["Hardware"]
        CAM_VIS["Basler RGB\n(GigE · PoE)"]
        CAM_LWIR["FLIR A68 Thermal\n(GigE · PoE)"]
        LIDAR["Ouster OS0-128U\n(GigE · internal-osc)"]
        DHT_HW["DHT22\n(NodeMCU · USB)"]
        GNSS_HW["u-blox 7 GPS\n(USB)"]
        ODO_HW["IMU / odometry"]
    end

    subgraph DRV["C++ Camera Drivers — core/ (ROS-independent)"]
        direction TB
        VIS_CORE["basler_adapter.cpp\n(Pylon SDK)"]
        LWIR_CORE["flir_adapter.cpp\n(Spinnaker SDK)"]
    end

    subgraph ROS_DRV["ROS 2 Thin Layer — camera_handler_node.cpp"]
        VIS_NODE["basler_camera_handler"]
        LWIR_NODE["flir_camera_handler"]
    end

    subgraph EXT["External ROS 2 Drivers"]
        OUSTER_DRV["Ouster driver\n(ouster_ros)"]
        GNSS_DRV["GNSS driver"]
        ODO_DRV["Odometry / EKF"]
        DHT_NODE["dht22_node.py\n(temperature_driver)"]
        RECAL["ouster_recal_node\n(hitos_setup · internal-osc → wall-clock)"]
        SMOOTH["gps_smoother\n(hitos_setup · GPS-only window filter)"]
    end

    subgraph CROP["C++ FOV Crop"]
        PC_CROP["pointcloud_crop_node"]
        IMG_CROP["image_crop_node ×4"]
    end

    subgraph BUF["Python Buffer Handlers — buffer_handler_node.py"]
        B_VIS["Buffer Visible\n(store_all)"]
        B_LWIR["Buffer LWIR"]
        B_LIDAR["Buffer LIDAR ×5\n(sync → crop → store)"]
        B_GNSS["Buffer GNSS"]
        B_ODO["Buffer Odom"]
        B_DHT["Buffer DHT22"]
    end

    subgraph DISK["Disk — configured_path / mult_session /"]
        D_VIS[("visible/")]
        D_LWIR[("lwir/")]
        D_LIDAR[("lidar_range/ · lidar_reflec/\nlidar_signal/ · lidar_nearir/\nlidar_pointcloud/")]
        D_GNSS[("gnss/")]
        D_ODO[("odom/")]
        D_DHT[("dht22/")]
    end

    subgraph CTRL["Control — hitos_web_manager :5050"]
        GUI["Web dashboard"]
        REC(["recording_enabled\nBool"])
    end

    CAM_VIS  --> VIS_CORE  --> VIS_NODE
    CAM_LWIR --> LWIR_CORE --> LWIR_NODE
    DHT_HW  --> DHT_NODE
    GNSS_HW --> GNSS_DRV
    ODO_HW  --> ODO_DRV
    LIDAR   --> OUSTER_DRV
    OUSTER_DRV --> PC_CROP & IMG_CROP

    VIS_NODE   --> B_VIS
    LWIR_NODE  --> B_LWIR
    PC_CROP & IMG_CROP --> B_LIDAR
    GNSS_DRV   -->|/gnss/fix| SMOOTH
    ODO_DRV    --> B_ODO
    DHT_NODE   --> B_DHT

    VIS_NODE -.->|master trigger| B_LWIR
    VIS_NODE -.->|master trigger| B_LIDAR
    VIS_NODE -.->|master trigger| B_GNSS
    VIS_NODE -.->|master trigger| B_ODO
    VIS_NODE -.->|master trigger| B_DHT

    B_VIS   --> D_VIS
    B_LWIR  --> D_LWIR
    B_LIDAR --> D_LIDAR
    B_GNSS  --> D_GNSS
    B_ODO   --> D_ODO
    B_DHT   --> D_DHT

    GUI --> REC
    REC -.->|enable · disable| BUF

    OUSTER_DRV -->|/ouster/imu · 100 Hz| RECAL
    RECAL -.->|/ouster/clock_offset| B_LIDAR
    SMOOTH -->|/gnss/fix_smoothed| B_GNSS

    style HW       fill:#e8e8e8,stroke:#888,color:#333
    style DRV      fill:#a8d5ff,stroke:#4a90d9,color:#1a3a5c
    style ROS_DRV  fill:#c5e0f7,stroke:#4a90d9,color:#1a3a5c
    style EXT      fill:#ddd,stroke:#999,color:#555
    style CROP     fill:#a8d5ff,stroke:#4a90d9,color:#1a3a5c
    style BUF      fill:#b8e6b8,stroke:#5aa55a,color:#1a3a1a
    style DISK     fill:#f5deb3,stroke:#c8a050,color:#5a4010
    style CTRL     fill:#ffcc99,stroke:#e6a040,color:#5a3510

    linkStyle 17,18,19,20,21 stroke:#e05050,stroke-width:2,stroke-dasharray:5
    linkStyle 29 stroke:#e6a040,stroke-width:2,stroke-dasharray:5
    linkStyle 31 stroke:#2a9d8f,stroke-width:2,stroke-dasharray:5
```

**Legend**: <span style="color:#4a90d9">blue</span> = C++ nodes · <span style="color:#5aa55a">green</span> = Python buffers · <span style="color:#c8a050">tan</span> = disk · <span style="color:#e6a040">orange dashed</span> = recording control · <span style="color:#e05050">red dashed</span> = master trigger · gray = external drivers / hardware

<details>
<summary>Alternative applications (ROS 1 era)</summary>

The package was originally deployed on an Ackermann/Husky robot. The architecture is identical — same driver/buffer/storage design — but without PTP or FOV crop. The GUI was a standalone Flask + SocketIO app (`multiespectral_acquire_gui`) on port 5051/5052.

### Multiespectral (Husky)

```mermaid
flowchart LR
    subgraph HW["Hardware"]
        CAM_VIS["Basler RGB\n(GigE)"]
        CAM_LWIR["FLIR Thermal\n(GigE)"]
        LIDAR["Ouster LIDAR"]
        DHT_HW["DHT22\n(NodeMCU · USB)"]
        GNSS_HW["GNSS receiver"]
        ODO_HW["Wheel encoders\n+ IMU"]
    end

    subgraph DRV["C++ Camera Drivers — core/ (ROS-independent)"]
        direction TB
        VIS_CORE["basler_adapter.cpp\n(Pylon SDK)"]
        LWIR_CORE["flir_adapter.cpp\n(Spinnaker SDK)"]
    end

    subgraph ROS_DRV["ROS Thin Layer — camera_handler_node.cpp"]
        VIS_NODE["basler_camera_handler"]
        LWIR_NODE["flir_camera_handler"]
    end

    subgraph EXT["External ROS Drivers"]
        OUSTER_DRV["Ouster driver\n(ouster_ros)"]
        GNSS_DRV["GNSS driver"]
        ODO_DRV["Odometry"]
        DHT_NODE["dht22_node.py\n(temperature_driver)"]
    end

    subgraph CROP["C++ FOV Crop"]
        PC_CROP["pointcloud_crop_node"]
        IMG_CROP["image_crop_node ×4"]
    end

    subgraph BUF["Python Buffer Handlers — buffer_handler_node.py"]
        B_VIS["Buffer Visible\n(store_all)"]
        B_LWIR["Buffer LWIR"]
        B_LIDAR["Buffer LIDAR ×5\n(sync → crop → store)"]
        B_GNSS["Buffer GNSS"]
        B_ODO["Buffer Odom"]
        B_DHT["Buffer DHT22"]
    end

    subgraph DISK["Disk — configured_path / mult_session /"]
        D_VIS[("visible/")]
        D_LWIR[("lwir/")]
        D_LIDAR[("lidar_range/ · lidar_reflec/\nlidar_signal/ · lidar_nearir/\nlidar_pointcloud/")]
        D_GNSS[("gnss/")]
        D_ODO[("odom/")]
        D_DHT[("dht22/")]
    end

    subgraph CTRL["Control — GUI :5051"]
        GUI["Web GUI"]
        REC(["recording_enabled\nBool"])
    end

    CAM_VIS  --> VIS_CORE  --> VIS_NODE
    CAM_LWIR --> LWIR_CORE --> LWIR_NODE
    DHT_HW  --> DHT_NODE
    GNSS_HW --> GNSS_DRV
    ODO_HW  --> ODO_DRV
    LIDAR   --> OUSTER_DRV
    OUSTER_DRV --> PC_CROP & IMG_CROP

    VIS_NODE  --> B_VIS
    LWIR_NODE --> B_LWIR
    PC_CROP & IMG_CROP --> B_LIDAR
    GNSS_DRV  --> B_GNSS
    ODO_DRV   --> B_ODO
    DHT_NODE  --> B_DHT

    VIS_NODE -.->|master trigger| B_LWIR
    VIS_NODE -.->|master trigger| B_LIDAR
    VIS_NODE -.->|master trigger| B_GNSS
    VIS_NODE -.->|master trigger| B_ODO
    VIS_NODE -.->|master trigger| B_DHT

    B_VIS   --> D_VIS
    B_LWIR  --> D_LWIR
    B_LIDAR --> D_LIDAR
    B_GNSS  --> D_GNSS
    B_ODO   --> D_ODO
    B_DHT   --> D_DHT

    GUI --> REC
    REC -.->|enable · disable| BUF

    style HW       fill:#e8e8e8,stroke:#888,color:#333
    style DRV      fill:#a8d5ff,stroke:#4a90d9,color:#1a3a5c
    style ROS_DRV  fill:#c5e0f7,stroke:#4a90d9,color:#1a3a5c
    style EXT      fill:#ddd,stroke:#999,color:#555
    style CROP     fill:#a8d5ff,stroke:#4a90d9,color:#1a3a5c
    style BUF      fill:#b8e6b8,stroke:#5aa55a,color:#1a3a1a
    style DISK     fill:#f5deb3,stroke:#c8a050,color:#5a4010
    style CTRL     fill:#ffcc99,stroke:#e6a040,color:#5a3510

    linkStyle 16,17,18,19,20 stroke:#e05050,stroke-width:2,stroke-dasharray:5
    linkStyle 28 stroke:#e6a040,stroke-width:2,stroke-dasharray:5
```

### Fisheye (Husky)

```mermaid
flowchart LR
    subgraph HW["Hardware"]
        CAM_FRONT["Basler Frontal\n(GigE)"]
        CAM_REAR["Basler Rear\n(GigE)"]
        LIDAR["Ouster LIDAR"]
        GNSS_HW["GNSS receiver"]
        ODO_HW["Wheel encoders\n+ IMU"]
    end

    subgraph DRV["C++ Camera Drivers — core/"]
        VIS_CORE_F["basler_adapter.cpp"]
        VIS_CORE_R["basler_adapter.cpp"]
    end

    subgraph ROS_DRV["ROS Thin Layer"]
        FRONT_NODE["frontal_camera_handler"]
        REAR_NODE["rear_camera_handler"]
    end

    subgraph EXT["External ROS Drivers"]
        OUSTER_DRV["Ouster driver\n(ouster_ros)"]
        GNSS_DRV["GNSS driver"]
        ODO_DRV["Odometry"]
    end

    subgraph BUF["Python Buffer Handlers"]
        B_FRONT["Buffer Frontal\n(store_all)"]
        B_REAR["Buffer Rear"]
        B_PC["Buffer Pointcloud"]
        B_GNSS["Buffer GNSS"]
        B_ODO["Buffer Odom"]
    end

    subgraph DISK["Disk — configured_path / pr_session /"]
        D_FRONT[("frontal/")]
        D_REAR[("rear/")]
        D_PC[("pointcloud/")]
        D_GNSS[("gnss/")]
        D_ODO[("odom/")]
    end

    subgraph CTRL["Control — GUI :5052"]
        GUI["Web GUI"]
        REC(["recording_enabled\nBool"])
    end

    CAM_FRONT --> VIS_CORE_F --> FRONT_NODE
    CAM_REAR  --> VIS_CORE_R --> REAR_NODE
    GNSS_HW --> GNSS_DRV
    ODO_HW  --> ODO_DRV
    LIDAR   --> OUSTER_DRV

    FRONT_NODE --> B_FRONT
    REAR_NODE  --> B_REAR
    OUSTER_DRV --> B_PC
    GNSS_DRV   --> B_GNSS
    ODO_DRV    --> B_ODO

    FRONT_NODE -.->|master trigger| B_REAR
    FRONT_NODE -.->|master trigger| B_PC
    FRONT_NODE -.->|master trigger| B_GNSS
    FRONT_NODE -.->|master trigger| B_ODO

    B_FRONT --> D_FRONT
    B_REAR  --> D_REAR
    B_PC    --> D_PC
    B_GNSS  --> D_GNSS
    B_ODO   --> D_ODO

    GUI --> REC
    REC -.->|enable · disable| BUF

    style HW      fill:#e8e8e8,stroke:#888,color:#333
    style DRV     fill:#a8d5ff,stroke:#4a90d9,color:#1a3a5c
    style ROS_DRV fill:#c5e0f7,stroke:#4a90d9,color:#1a3a5c
    style EXT     fill:#ddd,stroke:#999,color:#555
    style BUF     fill:#b8e6b8,stroke:#5aa55a,color:#1a3a1a
    style DISK    fill:#f5deb3,stroke:#c8a050,color:#5a4010
    style CTRL    fill:#ffcc99,stroke:#e6a040,color:#5a3510

    linkStyle 12,13,14,15 stroke:#e05050,stroke-width:2,stroke-dasharray:5
    linkStyle 22 stroke:#e6a040,stroke-width:2,stroke-dasharray:5
```

**Legend** (both diagrams): <span style="color:#4a90d9">blue</span> = C++ nodes · <span style="color:#5aa55a">green</span> = Python buffers · <span style="color:#c8a050">tan</span> = disk · <span style="color:#e6a040">orange dashed</span> = recording control · <span style="color:#e05050">red dashed</span> = master trigger · gray = external drivers / hardware

</details>

---

## Quick Start

On HITOS, acquisition is managed by systemd — see `hitos_setup` for service controls. For manual launch:

```bash
# Multiespectral
ros2 launch multiespectral_acquire multiespectral_launch.py session_folder:="my_session"
```

Recording control:
```bash
ros2 topic pub --once /Multiespectral/recording_enabled std_msgs/msg/Bool "data: true"
ros2 topic pub --once /Multiespectral/recording_enabled std_msgs/msg/Bool "data: false"
```

### Web GUI

In practice recording is driven from the responsive web dashboard (`multiespectral_acquire_gui`), which also shows live camera feeds and per-topic frame rates:

<p align="center">
  <img src="media/acquisition_gui_desktop_dark.png" width="540" alt="Acquisition web GUI — desktop"/>
</p>

See [multiespectral_acquire_gui/README.md](multiespectral_acquire_gui/README.md) for the full set of desktop and mobile screenshots (light/dark themes) and configuration.

---

## Output Structure

### Multiespectral
```
<configured_path>/mult_<session>/
├── visible/           # PNG + YAML per frame (master, all stored)
├── lwir/              # PNG + YAML (synced to visible master)
├── lidar_range/       # Cropped range depth images
├── lidar_reflec/      # Cropped reflectivity images
├── lidar_signal/      # Cropped signal strength images
├── lidar_nearir/      # Cropped near-IR images
├── lidar_pointcloud/  # BIN + YAML (cropped 3D point clouds)
├── gnss/              # YAML (GPS fixes, synced to master)
├── odom/              # YAML (odometry, synced to master)
├── dht22/             # YAML (temperature + humidity, synced to master)
├── tf_static.yaml     # Static TF tree snapshot at recording start
└── ouster_metadata.json  # Ouster sensor metadata snapshot at recording start
```

---

## Dependencies

| Library | Required by | Notes |
|---------|-------------|-------|
| [Pylon SDK](https://www.baslerweb.com/en/software/pylon/) | `basler_camera_handler` | Proprietary, install manually |
| [Spinnaker SDK](https://www.flir.com/products/spinnaker-sdk/) | `flir_camera_handler` | Proprietary, install manually |
| OpenCV | All image nodes | `apt install libopencv-dev` |

For per-package details see:
- [multiespectral_acquire/README.md](multiespectral_acquire/README.md) — drivers, timestamp calibration, buffer synchronization, crash recovery
- [multiespectral_acquire_gui/README.md](multiespectral_acquire_gui/README.md) — web dashboard for recording control and live monitoring
- [temperature_driver/README.md](temperature_driver/README.md) — DHT22 sensor setup

---

<details>
<summary>Historical architecture diagrams (original Husky + Fisheye deployments)</summary>

The package was originally deployed on an Ackermann/Husky robot with a different sensor suite. The architecture is analogous to HITOS — same driver/buffer/storage design — but without PTP or FOV crop.

### Multiespectral (Husky)

```mermaid
flowchart LR
    subgraph HW["Hardware"]
        CAM_VIS["Basler RGB\n(GigE)"]
        CAM_LWIR["FLIR Thermal\n(GigE)"]
        LIDAR["Ouster LIDAR"]
        DHT_HW["DHT22\n(NodeMCU · USB)"]
        GNSS_HW["GNSS receiver"]
        ODO_HW["Wheel encoders\n+ IMU"]
    end

    subgraph DRV["C++ Camera Drivers — core/ (ROS-independent)"]
        direction TB
        VIS_CORE["basler_adapter.cpp\n(Pylon SDK)"]
        LWIR_CORE["flir_adapter.cpp\n(Spinnaker SDK)"]
    end

    subgraph ROS_DRV["ROS Thin Layer — camera_handler_node.cpp"]
        VIS_NODE["basler_camera_handler"]
        LWIR_NODE["flir_camera_handler"]
    end

    subgraph EXT["External ROS Drivers"]
        OUSTER_DRV["Ouster driver\n(ouster_ros)"]
        GNSS_DRV["GNSS driver"]
        ODO_DRV["Odometry"]
        DHT_NODE["dht22_node.py\n(temperature_driver)"]
    end

    subgraph CROP["C++ FOV Crop"]
        PC_CROP["pointcloud_crop_node"]
        IMG_CROP["image_crop_node ×4"]
    end

    subgraph BUF["Python Buffer Handlers — buffer_handler_node.py"]
        B_VIS["Buffer Visible\n(store_all)"]
        B_LWIR["Buffer LWIR"]
        B_LIDAR["Buffer LIDAR ×5\n(sync → crop → store)"]
        B_GNSS["Buffer GNSS"]
        B_ODO["Buffer Odom"]
        B_DHT["Buffer DHT22"]
    end

    subgraph DISK["Disk — configured_path / mult_session /"]
        D_VIS[("visible/")]
        D_LWIR[("lwir/")]
        D_LIDAR[("lidar_range/ · lidar_reflec/\nlidar_signal/ · lidar_nearir/\nlidar_pointcloud/")]
        D_GNSS[("gnss/")]
        D_ODO[("odom/")]
        D_DHT[("dht22/")]
    end

    subgraph CTRL["Control — GUI :5051"]
        GUI["Web GUI"]
        REC(["recording_enabled\nBool"])
    end

    CAM_VIS  --> VIS_CORE  --> VIS_NODE
    CAM_LWIR --> LWIR_CORE --> LWIR_NODE
    DHT_HW  --> DHT_NODE
    GNSS_HW --> GNSS_DRV
    ODO_HW  --> ODO_DRV
    LIDAR   --> OUSTER_DRV
    OUSTER_DRV --> PC_CROP & IMG_CROP

    VIS_NODE  --> B_VIS
    LWIR_NODE --> B_LWIR
    PC_CROP & IMG_CROP --> B_LIDAR
    GNSS_DRV  --> B_GNSS
    ODO_DRV   --> B_ODO
    DHT_NODE  --> B_DHT

    VIS_NODE -.->|master trigger| B_LWIR
    VIS_NODE -.->|master trigger| B_LIDAR
    VIS_NODE -.->|master trigger| B_GNSS
    VIS_NODE -.->|master trigger| B_ODO
    VIS_NODE -.->|master trigger| B_DHT

    B_VIS   --> D_VIS
    B_LWIR  --> D_LWIR
    B_LIDAR --> D_LIDAR
    B_GNSS  --> D_GNSS
    B_ODO   --> D_ODO
    B_DHT   --> D_DHT

    GUI --> REC
    REC -.->|enable · disable| BUF

    style HW       fill:#e8e8e8,stroke:#888,color:#333
    style DRV      fill:#a8d5ff,stroke:#4a90d9,color:#1a3a5c
    style ROS_DRV  fill:#c5e0f7,stroke:#4a90d9,color:#1a3a5c
    style EXT      fill:#ddd,stroke:#999,color:#555
    style CROP     fill:#a8d5ff,stroke:#4a90d9,color:#1a3a5c
    style BUF      fill:#b8e6b8,stroke:#5aa55a,color:#1a3a1a
    style DISK     fill:#f5deb3,stroke:#c8a050,color:#5a4010
    style CTRL     fill:#ffcc99,stroke:#e6a040,color:#5a3510

    linkStyle 17,18,19,20,21 stroke:#e05050,stroke-width:2,stroke-dasharray:5
    linkStyle 29 stroke:#e6a040,stroke-width:2,stroke-dasharray:5
```

### Fisheye (Husky)

```mermaid
flowchart LR
    subgraph HW["Hardware"]
        CAM_FRONT["Basler Frontal\n(GigE)"]
        CAM_REAR["Basler Rear\n(GigE)"]
        LIDAR["Ouster LIDAR"]
        GNSS_HW["GNSS receiver"]
        ODO_HW["Wheel encoders\n+ IMU"]
    end

    subgraph DRV["C++ Camera Drivers — core/"]
        VIS_CORE_F["basler_adapter.cpp"]
        VIS_CORE_R["basler_adapter.cpp"]
    end

    subgraph ROS_DRV["ROS Thin Layer"]
        FRONT_NODE["frontal_camera_handler"]
        REAR_NODE["rear_camera_handler"]
    end

    subgraph EXT["External ROS Drivers"]
        OUSTER_DRV["Ouster driver\n(ouster_ros)"]
        GNSS_DRV["GNSS driver"]
        ODO_DRV["Odometry"]
    end

    subgraph BUF["Python Buffer Handlers"]
        B_FRONT["Buffer Frontal\n(store_all)"]
        B_REAR["Buffer Rear"]
        B_PC["Buffer Pointcloud"]
        B_GNSS["Buffer GNSS"]
        B_ODO["Buffer Odom"]
    end

    subgraph DISK["Disk — configured_path / pr_session /"]
        D_FRONT[("frontal/")]
        D_REAR[("rear/")]
        D_PC[("pointcloud/")]
        D_GNSS[("gnss/")]
        D_ODO[("odom/")]
    end

    subgraph CTRL["Control — GUI :5052"]
        GUI["Web GUI"]
        REC(["recording_enabled\nBool"])
    end

    CAM_FRONT --> VIS_CORE_F --> FRONT_NODE
    CAM_REAR  --> VIS_CORE_R --> REAR_NODE
    GNSS_HW --> GNSS_DRV
    ODO_HW  --> ODO_DRV
    LIDAR   --> OUSTER_DRV

    FRONT_NODE --> B_FRONT
    REAR_NODE  --> B_REAR
    OUSTER_DRV --> B_PC
    GNSS_DRV   --> B_GNSS
    ODO_DRV    --> B_ODO

    FRONT_NODE -.->|master trigger| B_REAR
    FRONT_NODE -.->|master trigger| B_PC
    FRONT_NODE -.->|master trigger| B_GNSS
    FRONT_NODE -.->|master trigger| B_ODO

    B_FRONT --> D_FRONT
    B_REAR  --> D_REAR
    B_PC    --> D_PC
    B_GNSS  --> D_GNSS
    B_ODO   --> D_ODO

    GUI --> REC
    REC -.->|enable · disable| BUF

    style HW      fill:#e8e8e8,stroke:#888,color:#333
    style DRV     fill:#a8d5ff,stroke:#4a90d9,color:#1a3a5c
    style ROS_DRV fill:#c5e0f7,stroke:#4a90d9,color:#1a3a5c
    style EXT     fill:#ddd,stroke:#999,color:#555
    style BUF     fill:#b8e6b8,stroke:#5aa55a,color:#1a3a1a
    style DISK    fill:#f5deb3,stroke:#c8a050,color:#5a4010
    style CTRL    fill:#ffcc99,stroke:#e6a040,color:#5a3510

    linkStyle 12,13,14,15 stroke:#e05050,stroke-width:2,stroke-dasharray:5
    linkStyle 22 stroke:#e6a040,stroke-width:2,stroke-dasharray:5
```

</details>
