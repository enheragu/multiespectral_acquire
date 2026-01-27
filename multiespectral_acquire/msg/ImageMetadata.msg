# Image metadata for multiespectral acquisition
Header header  # ROS timestamp = camera_timestamp

# Timestamps (all in nanoseconds)
uint64 camera_timestamp       # Hardware timestamp from camera (PTP or calibrated)
uint64 exposure_time          # Exposure duration
uint64 half_exposure_timestamp # camera_timestamp + exposure/2 (optimal sync point)

# Frame info
uint64 frame_counter
float64 gain
uint32 width
uint32 height
string pixel_format

# Dataset info
string timetag
string img_name
string img_pair_name
string dataset_name
