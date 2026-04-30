# GNSS low bandwidth position message for efficient telemetry transmission
# Contains basic GNSS position and velocity information

uint64 timestamp              # time since system start (microseconds)
uint64 time_utc_usec          # Timestamp (microseconds, UTC) from GPS module

float64 latitude_deg          # Latitude in degrees (WGS84)
float64 longitude_deg         # Longitude in degrees (WGS84)
float32 altitude_msl_m        # Altitude above mean sea level (meters)
float32 groundspeed_m         # Ground speed (m/s)

# TOPICS gnss_low_bandwidth_position
