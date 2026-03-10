# Multiespectral Acquire

Generic metapackage for **synchronized multi-sensor data acquisition** on mobile robots. The drivers, buffer synchronization pipeline, storage system and web GUI are all configurable via ROS parameters and launch files — any combination of cameras and sensors can be assembled.

| Package | Description |
|---------|-------------|
| [multiespectral_acquire](multiespectral_acquire/) | Camera drivers (Basler, FLIR), timestamp synchronization, buffer/storage pipeline, FOV crop |
| [multiespectral_acquire_gui](multiespectral_acquire_gui/) | Flask + SocketIO web GUI for real-time acquisition control |
| [temperature_driver](temperature_driver/) | DHT22 temperature/humidity sensor via NodeMCU (ESP8266) |

Two ready-to-use configurations are provided as examples:

| Configuration | Launch file | Master | Slave sensors | Namespace |
|---------------|-------------|--------|---------------|-----------|
| **Multiespectral** | `multiespectral_launch.launch` | Basler RGB (1 Hz) | FLIR LWIR (30 Hz, PTP), Ouster LIDAR (cropped), GNSS, odom, DHT22 | `/Multiespectral` |
| **Fisheye** | `fisheye_launch.launch` | Basler frontal (1 Hz) | Basler rear (4 Hz), Ouster pointcloud (raw), GNSS, odom | `/Fisheye` |

---

## Architecture — Multiespectral Setup

```mermaid
flowchart LR
    subgraph HW["Hardware"]
        CAM_VIS["Basler RGB<br/>(GigE)"]
        CAM_LWIR["FLIR Thermal<br/>(GigE)"]
        LIDAR["Ouster LIDAR"]
        DHT_HW["DHT22<br/>(NodeMCU · USB)"]
        GNSS_HW["GNSS receiver"]
        ODO_HW["Wheel encoders<br/>+ IMU"]
    end

    subgraph DRV["C++ Camera Drivers — core/ (ROS-independent)"]
        direction TB
        VIS_CORE["basler_adapter.cpp<br/>(Pylon SDK)"]
        LWIR_CORE["flir_adapter.cpp<br/>(Spinnaker SDK)"]
    end

    subgraph ROS_DRV["ROS Thin Layer — camera_handler_node.cpp"]
        VIS_NODE["basler_camera_handler"]
        LWIR_NODE["flir_camera_handler"]
    end

    subgraph EXT["External ROS Drivers"]
        OUSTER_DRV["Ouster driver<br/>(ouster_ros)"]
        GNSS_DRV["GNSS driver"]
        ODO_DRV["Odometry"]
        DHT_NODE["dht22_node.py<br/>(temperature_driver)"]
    end

    subgraph CROP["C++ FOV Crop"]
        PC_CROP["pointcloud_crop_node"]
        IMG_CROP["image_crop_node ×4"]
    end

    subgraph BUF["Python Buffer Handlers — buffer_handler_node.py"]
        B_VIS["Buffer Visible<br/>(store_all)"]
        B_LWIR["Buffer LWIR"]
        B_LIDAR["Buffer LIDAR ×5<br/>(sync → crop → store)"]
        B_GNSS["Buffer GNSS"]
        B_ODO["Buffer Odom"]
        B_DHT["Buffer DHT22"]
    end

    subgraph DISK["Disk — configured_path / mult_session /"]
        D_VIS[("visible/")]
        D_LWIR[("lwir/")]
        D_LIDAR[("lidar_range/ · lidar_reflec/<br/>lidar_signal/ · lidar_nearir/<br/>lidar_pointcloud/")]
        D_GNSS[("gnss/")]
        D_ODO[("odom/")]
        D_DHT[("dht22/")]
    end

    subgraph CTRL["Control — GUI :5051"]
        GUI["Web GUI"]
        REC(["recording_enabled<br/>Bool"])
    end

    CAM_VIS --> VIS_CORE --> VIS_NODE
    CAM_LWIR --> LWIR_CORE --> LWIR_NODE
    DHT_HW --> DHT_NODE
    GNSS_HW --> GNSS_DRV
    ODO_HW --> ODO_DRV
    LIDAR --> OUSTER_DRV
    OUSTER_DRV --> PC_CROP & IMG_CROP

    VIS_NODE --> B_VIS
    LWIR_NODE --> B_LWIR
    PC_CROP & IMG_CROP --> B_LIDAR
    GNSS_DRV --> B_GNSS
    ODO_DRV --> B_ODO
    DHT_NODE --> B_DHT

    VIS_NODE -.->|master trigger| B_LWIR
    VIS_NODE -.->|master trigger| B_LIDAR
    VIS_NODE -.->|master trigger| B_GNSS
    VIS_NODE -.->|master trigger| B_ODO
    VIS_NODE -.->|master trigger| B_DHT

    B_VIS --> D_VIS
    B_LWIR --> D_LWIR
    B_LIDAR --> D_LIDAR
    B_GNSS --> D_GNSS
    B_ODO --> D_ODO
    B_DHT --> D_DHT

    GUI --> REC
    REC -.->|enable · disable| BUF

    style HW fill:#e8e8e8,stroke:#888,color:#333
    style DRV fill:#a8d5ff,stroke:#4a90d9,color:#1a3a5c
    style ROS_DRV fill:#c5e0f7,stroke:#4a90d9,color:#1a3a5c
    style EXT fill:#ddd,stroke:#999,color:#555
    style CROP fill:#a8d5ff,stroke:#4a90d9,color:#1a3a5c
    style BUF fill:#b8e6b8,stroke:#5aa55a,color:#1a3a1a
    style DISK fill:#f5deb3,stroke:#c8a050,color:#5a4010
    style CTRL fill:#ffcc99,stroke:#e6a040,color:#5a3510

    linkStyle 17,18,19,20,21 stroke:#e05050,stroke-width:2,stroke-dasharray:5
    linkStyle 29 stroke:#e6a040,stroke-width:2,stroke-dasharray:5
```

## Architecture — Fisheye Setup

```mermaid
flowchart LR
    subgraph HW["Hardware"]
        CAM_FRONT["Basler Frontal<br/>(GigE)"]
        CAM_REAR["Basler Rear<br/>(GigE)"]
        LIDAR["Ouster LIDAR"]
        GNSS_HW["GNSS receiver"]
        ODO_HW["Wheel encoders<br/>+ IMU"]
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
        OUSTER_DRV["Ouster driver<br/>(ouster_ros)"]
        GNSS_DRV["GNSS driver"]
        ODO_DRV["Odometry"]
    end

    subgraph BUF["Python Buffer Handlers"]
        B_FRONT["Buffer Frontal<br/>(store_all)"]
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
        REC(["recording_enabled<br/>Bool"])
    end

    CAM_FRONT --> VIS_CORE_F --> FRONT_NODE
    CAM_REAR --> VIS_CORE_R --> REAR_NODE
    GNSS_HW --> GNSS_DRV
    ODO_HW --> ODO_DRV
    LIDAR --> OUSTER_DRV

    FRONT_NODE --> B_FRONT
    REAR_NODE --> B_REAR
    OUSTER_DRV --> B_PC
    GNSS_DRV --> B_GNSS
    ODO_DRV --> B_ODO

    FRONT_NODE -.->|master trigger| B_REAR
    FRONT_NODE -.->|master trigger| B_PC
    FRONT_NODE -.->|master trigger| B_GNSS
    FRONT_NODE -.->|master trigger| B_ODO

    B_FRONT --> D_FRONT
    B_REAR --> D_REAR
    B_PC --> D_PC
    B_GNSS --> D_GNSS
    B_ODO --> D_ODO

    GUI --> REC
    REC -.->|enable · disable| BUF

    style HW fill:#e8e8e8,stroke:#888,color:#333
    style DRV fill:#a8d5ff,stroke:#4a90d9,color:#1a3a5c
    style ROS_DRV fill:#c5e0f7,stroke:#4a90d9,color:#1a3a5c
    style EXT fill:#ddd,stroke:#999,color:#555
    style BUF fill:#b8e6b8,stroke:#5aa55a,color:#1a3a1a
    style DISK fill:#f5deb3,stroke:#c8a050,color:#5a4010
    style CTRL fill:#ffcc99,stroke:#e6a040,color:#5a3510

    linkStyle 12,13,14,15 stroke:#e05050,stroke-width:2,stroke-dasharray:5
    linkStyle 22 stroke:#e6a040,stroke-width:2,stroke-dasharray:5
```

**Legend** (both diagrams): <span style="color:#4a90d9">blue</span> = C++ nodes (core + ROS layer) · <span style="color:#5aa55a">green</span> = Python buffers · <span style="color:#c8a050">tan</span> = disk · <span style="color:#e6a040">orange dashed</span> = recording control · <span style="color:#e05050">red dashed</span> = master trigger · gray = external drivers / hardware

**Key differences**: the multiespectral setup adds FOV crop nodes between the Ouster driver and the buffers (2-stage LIDAR pipeline), uses a FLIR thermal camera as slave with PTP, and includes the DHT22 sensor. The fisheye setup uses two Basler cameras (no PTP, no crop), stores raw pointclouds directly, and has no temperature sensor.

---

The camera drivers use a **two-layer architecture**: `core/` (pure C++, ROS-independent) handles all SDK interaction, while `camera_handler_node.cpp` is a thin ROS wrapper. See [multiespectral_acquire/README.md](multiespectral_acquire/README.md) for details. This makes migrating from ROS1 to ROS2 a matter of replacing only the thin wrapper.

**The master camera (Basler) drives all synchronization**: every time it captures a frame, each buffer handler finds the closest matching frame from its slave sensor within a configurable time window and stores the synchronized pair. The `recording_enabled` topic controls all buffers at once.

### Multiespectral GUI (desktop view, with LIDAR displayed)

<p align="center">
  <img src="media/desktop_lidar.png" width="700"/>
</p>

The GUI is responsive — works on desktop and mobile. See [multiespectral_acquire_gui/README.md](multiespectral_acquire_gui/README.md) for all views.

## Quick Start

### Multiespectral
```bash
# Auto-timestamped session
./multiespectral_acquire/scripts/launch_multiespectral_with_timestamp.sh

# Or manual
roslaunch multiespectral_acquire multiespectral_launch.launch session_folder:="my_session"

# GUI (port 5051)
roslaunch multiespectral_acquire_gui multiespectral_gui_launch.launch
```

### Fisheye
```bash
# Auto-timestamped session
roslaunch multiespectral_acquire fisheye_launch.launch session_folder:="my_fisheye_session"

# GUI (port 5052)
roslaunch multiespectral_acquire_gui fisheye_gui_launch.launch
```

### Recording control
```bash
# Via topic
rostopic pub /<namespace>/recording_enabled std_msgs/Bool "data: true"
rostopic pub /<namespace>/recording_enabled std_msgs/Bool "data: false"

# Or use the web GUI
```

## Output Structure

Each session creates a folder with one subfolder per sensor type:

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
└── dht22/             # YAML (temperature + humidity, synced to master)
```

### Fisheye
```
<configured_path>/pr_<session>/
├── frontal/           # PNG + YAML per frame (master, all stored)
├── rear/              # PNG + YAML (synced to frontal master)
├── pointcloud/        # BIN + YAML (raw, synced to master)
├── gnss/              # YAML (synced to master)
└── odom/              # YAML (synced to master)
```

## Dependencies

| Library | Required by | Notes |
|---------|-------------|-------|
| [Pylon SDK](https://www.baslerweb.com/en/software/pylon/) | `basler_camera_handler` | Proprietary, install manually |
| [Spinnaker SDK](https://www.flir.com/products/spinnaker-sdk/) | `flir_camera_handler` | Proprietary, install manually |
| OpenCV | All image nodes | `apt install libopencv-dev` |
| Flask + SocketIO | GUI package | `pip install -r multiespectral_acquire_gui/requirements.txt` |

For details on each package, see:
- [multiespectral_acquire/README.md](multiespectral_acquire/README.md) — Drivers, timestamp calibration (PTP vs software), buffer synchronization
- [multiespectral_acquire_gui/README.md](multiespectral_acquire_gui/README.md) — Web GUI usage
- [temperature_driver/README.md](temperature_driver/README.md) — DHT22 sensor setup

## GUI

The GUI is **generic and configurable** via ROS parameters — camera names, topics, and port are all set in the launch file. Two preconfigured variants are provided:

| Variant | Port | Launch | Camera 1 | Camera 2 |
|---------|------|--------|----------|----------|
| Multiespectral | 5051 | `multiespectral_gui_launch.launch` | LWIR thermal | Visible RGB |
| Fisheye | 5052 | `fisheye_gui_launch.launch` | Frontal | Rear |

Features:
- Real-time recording status (IDLE / RECORDING REQUESTED / RECORDING / STOP REQUESTED)
- Live camera preview synced at master rate
- Dynamic LIDAR visibility (shows/hides based on topic availability)
- Frame rate monitoring and image counter per sensor

**Note:** GUI subscribes to `_sync` topics — images only appear when recording is active.

### Custom Configuration

```bash
roslaunch multiespectral_acquire_gui multiespectral_gui_launch.launch \
    gui_title:="My Custom GUI" \
    camera1_topic:="/MyNamespace/thermal/image_sync" \
    camera1_name:="Thermal IR" \
    camera2_topic:="/MyNamespace/rgb/image_sync" \
    camera2_name:="RGB Camera"
```

## GUI control in python with venv

The python venv can be created inside the workspace but needs to be ignored by catkin:
```sh
    source /opt/ros/noetic/setup.bash
    python3 -m venv .venv
    touch .venv/CATKIN_IGNORE
    source .venv/bin/activate
    pip install -r multiespectral_acquire_gui/requirements.txt
```

## Related Repositories

| Repository | Relation |
|------------|----------|
| [husky_manager](https://github.com/enheragu/husky_manager) | Main web dashboard (port 5050) — links to the camera GUIs, provides process management and sensor monitoring |
| [test_utils](https://github.com/enheragu/test_utils) | Systemd services (`multiespectral_cameras`, `fisheye_cameras`, `multiespectral_gui`, `fisheye_gui`) for auto-start, shell shortcuts (F8/F9), Conky status display |

