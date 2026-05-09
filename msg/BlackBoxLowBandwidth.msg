# BlackBox low bandwidth position message for efficient telemetry transmission
# Contains basic BlackBox position and velocity information

uint64 timestamp              # time since system start (microseconds)
uint64 time_utc_usec          # Timestamp (microseconds, UTC) from GPS module

float64 latitude_deg          # Latitude in degrees (WGS84)
float64 longitude_deg         # Longitude in degrees (WGS84)
float32 altitude_msl_m        # Altitude above mean sea level (meters)
float32 altitude_ellipsoid_m  # Altitude above ellipsoid (meters)
float32 relative_alt_m        # Relative altitude (meters)

float32 vn_m_s                # North velocity (m/s)
float32 ve_m_s                # East velocity (m/s)
float32 vd_m_s                # Down velocity (m/s)

float32 heading_rad	      # Euler yaw angle transforming the tangent plane relative to NED earth-fixed frame, -PI..+PI,  (radians)

uint8 satellites_visible      # Number of satellites visible

uint8 fix_type                # GPS fix type (1=none, 2=2D, 3=3D, 4=RTCM differential, 5=RTK float, 6=RTK fixed)

uint8 battery_remaining       # Battery remaining percentage (0-100%)

uint8 rssi                    # Signal strength indicator  （units="-dBm"）

# TOPICS black_box_low_bandwidth
