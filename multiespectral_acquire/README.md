# multiespectral_acquire

Generic ROS package for synchronized multi-sensor data acquisition: camera drivers, timestamp calibration, buffer-based synchronization and disk storage.

The package is **hardware-agnostic** — any combination of cameras and sensors can be configured via launch files. Two example configurations are provided:

| Configuration | Main | Followers | LIDAR pipeline |
|---|---|---|---|
| **Multiespectral** | Basler RGB (1 Hz) | FLIR LWIR (PTP), Ouster (cropped), GNSS, odom, DHT22 | 2-stage: sync → FOV crop → store |
| **Fisheye** | Basler frontal (1 Hz) | Basler rear (4 Hz), Ouster (raw), GNSS, odom | Direct: sync → store |

## Driver Architecture (two-layer design)

The C++ code is split into two layers so the core logic is **completely ROS-independent**:

```
src/
├── camera_handler_node.cpp          ← ROS1 thin layer (pub/sub, timers, params)
├── image_crop_node.cpp
├── pointcloud_crop_node.cpp
├── core/                            ← Layer 1 — NO ROS dependencies
│   ├── camera_drivers/
│   │   ├── camera_adapter.h/.cpp    ← Abstract base + free-function API
│   │   ├── basler_adapter.cpp       ← Pylon SDK implementation
│   │   ├── flir_adapter.cpp         ← Spinnaker SDK implementation
│   │   └── dummy_adapter.cpp        ← Test stub (no hardware)
│   └── utils/
│       ├── logging_utils.h          ← Abstract Logger interface
│       ├── image_metadata.h/.cpp    ← ImageMetadata struct + YAML serialization
│       ├── timed_frame_buffer.h     ← Self-adjusting ring buffer
│       └── timestamp_calibration.h  ← PTP/software timestamp calibration
└── ros_utils/
    └── ros_logger.h                 ← RosLogger : Logger → ROS_INFO/WARN/etc.
```

**Key design decisions:**

- **Compile-time vendor selection** — CMake links exactly one of `basler_adapter.cpp`, `flir_adapter.cpp`, or `dummy_adapter.cpp` per executable. Each provides the same free-function set (`initCamera()`, `acquireImage()`, `beginAcquisition()`, etc.).
- **`Logger` abstraction** — `core/utils/logging_utils.h` defines a pure-virtual `Logger` class. The ROS layer injects a `RosLogger` via `CameraAdapter::setLogger()`. The core never includes any ROS header.
- **`ImageMetadata` bridge** — a plain C++ struct that carries timestamps, exposure, gain, etc. It has a `ROSTimeNowCallback` slot so the ROS layer can inject `ros::Time::now()` without the core depending on ROS.
- **`CameraHandlerNode`** inherits `CameraAdapter` and adds only ROS plumbing: parameter reads, `image_transport` publishers, a `ros::Timer` callback that calls `grabImage()` and publishes.

**Migration path (ROS1 → ROS2):** only `camera_handler_node.cpp` and `ros_logger.h` need to be rewritten. The entire `core/` layer remains unchanged.

## Nodes

### C++ Camera Drivers

| Executable | Source | Camera | SDK |
|------------|--------|--------|-----|
| `basler_camera_handler` | `src/core/camera_drivers/basler_adapter.cpp` | Basler acA1600-60gc (RGB) | Pylon |
| `flir_camera_handler` | `src/core/camera_drivers/flir_adapter.cpp` | FLIR Boson (LWIR thermal) | Spinnaker |
| `dummy_camera_handler` | `src/core/camera_drivers/dummy_adapter.cpp` | Test pattern | — |

Each driver publishes `ImageWithMetadata` (`image` + `metadata` with hardware timestamp, exposure, gain, etc.) at a configurable frame rate via software trigger.

### C++ Crop Nodes

| Executable | Purpose |
|------------|---------|
| `pointcloud_crop_node` | Crops 3D point cloud by configurable FOV (angular or pixel-based) |
| `image_crop_node` | Crops LIDAR 2D images (range, reflec, signal, nearir) by FOV |

### Python Buffer Handler

`scripts/buffer_handler_node.py` is generic and type-agnostic (`rospy.AnyMsg`).

It has two modes:
1. **store_all** (no `main_topic`): republishes and optionally stores every incoming message.
2. **sync** (with `main_topic`): aligns follower data to the main trigger timestamp.

In **sync** mode it uses a simple double-buffer strategy:
1. **Main buffer (`TimedBuffer`)**: thread-safe ring buffer with recent follower messages (starts at 100 frames).
2. **Pending sync queue**: bounded queue (max 10) for main triggers that cannot be matched immediately.

Per main trigger:
1. Computes target timestamp using exposure midpoint.
2. Tries nearest match in main buffer within `max_time_diff`.
3. If matched: republishes to `<topic>_sync` and optionally stores to disk (PNG/BIN + YAML).
4. If not matched and buffer is already newer than target: drops that sync request (too late) and may grow main buffer (bounded) to absorb future late main arrivals.
5. If not matched and buffer is not newer yet: enqueues request in pending queue and retries it on each new follower message.
6. If pending queue is full: oldest pending request is dropped.

## Timestamp Synchronization

All sensors synchronize to the **visible camera (Basler)** as main reference. The sync point accounts for exposure:

$$t_{sync} = t_{camera} + \frac{t_{exposure}}{2}$$

### PTP Mode (preferred)

When a camera supports IEEE 1588 PTP and a PTP grandmaster is running on the network, the camera's hardware clock is synchronized directly. Timestamps are in the same time domain as the PC — no software correction needed.

**Startup sanity check**: At `beginAcquisition()` the driver captures 10 samples and compares camera timestamps against PC time. If more than half exceed **5 seconds** difference (indicating PTP never locked), it automatically falls back to software calibration and logs a warning. This check runs once at startup since PTP failures are typically initialization problems, not mid-run issues.

### Software Calibration (fallback)

For cameras without PTP (e.g., Basler acA1600 via GigE):

1. **Initial calibration** (startup): captures N samples (30 for Basler at ~1 Hz, 120 for FLIR at 30 Hz), fits a linear model via least-squares regression:
   $$t_{PC} = \text{offset} + \text{slope} \times t_{camera}$$
2. **Adaptive online calibration**: continuously refines offset (EMA, α=0.05) and slope (recalculated every 25 samples via regression on a sliding window of 50 samples)
3. **Clock step detection**: if NTP/chrony suddenly adjusts the system clock, 3+ consecutive same-sign anomalies trigger an immediate offset correction (80%) instead of waiting for gradual EMA convergence
4. **Drift detection**: if the error trend exceeds 2 ms/sample, slope recalculation becomes aggressive (every 10 samples, α=0.1)

### PTP Setup (host side)

```bash
sudo apt install linuxptp
ethtool -T <interface>  # Verify PTP support

# Run PTP grandmaster + sync system clock
sudo ptp4l -i <interface> -m -S -f /etc/linuxptp/ptp4l.conf
sudo phc2sys -c CLOCK_REALTIME -s <interface> -w -m
```

Example `ptp4l.conf`:
```ini
[global]
gmCapable         1
logSyncInterval   1
logAnnounceInterval 1
logMinDelayReqInterval 0
```

## Launch Files

| Launch | Description |
|--------|-------------|
| `multiespectral_launch.launch` | Main entry point: cameras + LIDAR crop + buffer handlers |
| `multiespectral_buffer_handlers.launch` | All buffer handler instances (included by main launch) |
| `multiespectral_lidar_crop.launch` | LIDAR FOV crop nodes (included by main launch) |
| `fisheye_launch.launch` | Alternative configuration for fisheye cameras |
| `fisheye_buffer_handlers.launch` | Buffer handlers for fisheye setup |

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `session_folder` | `test_session` | Name for the output folder |
| `dataset_output_path` | `/media/administrator/data/images_eeha` | Base storage path |
| `output_frame_rate` | `1` | Acquisition rate (Hz) |
| `visible_use_ptp` | `false` | Enable PTP for Basler (needs hardware support) |
| `lwir_use_ptp` | `true` | Enable PTP for FLIR Boson |
| Buffer `max_time_diff` | `0.4` (LWIR), `0.3` (LIDAR) | Max sync tolerance (seconds) |

## Messages

| Message | Fields |
|---------|--------|
| `ImageWithMetadata` | `sensor_msgs/Image image` + `ImageMetadata metadata` |
| `ImageMetadata` | `camera_timestamp`, `exposure_time`, `half_exposure_timestamp`, `frame_counter`, `gain`, `width`, `height`, `pixel_format`, `timetag`, `img_name`, `img_pair_name`, `dataset_name` |
| `TriggerStamp` | Trigger synchronization stamp |

## Ouster LIDAR Images

The Ouster driver (`ouster_ros`, launched externally by `sensors_manager`) publishes 4 image representations per scan:

| Type | Topic | Content |
|------|-------|---------|
| Range | `/ouster/range_image` | Depth map (distance per point) |
| Reflectivity | `/ouster/reflec_image` | Laser return intensity |
| Signal | `/ouster/signal_image` | Photon count / signal strength |
| Near-IR | `/ouster/nearir_image` | Ambient infrared light |

All are cropped by `image_crop_node` to match the cameras' FOV before storage.
