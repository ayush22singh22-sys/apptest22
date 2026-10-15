# Road Monitor (Android app)
Records accelerometer + GPS + tilt to CSV, detects bumps / potholes on the phone, and sends them to the FastAPI backend on the laptop.
Build: push this folder to a GitHub repo -> Actions tab -> "Build APK" -> download artifact road-monitor-apk -> install app-debug.apk.
Drive files on the phone: Android/data/com.roadmonitor/files/drives/  (<id>_*.csv, <id>_events.csv, <id>_markers.csv, <id>_meta.json)

CSV columns: timestamp_ms,ax,ay,az,lat,lon,speed_kmh,gps_accuracy_m,bearing_deg,gps_age_ms,pitch_deg,roll_deg,vert_ms2
 - pitch/roll: tilt from the gravity direction (phone flat = 0/0, phone upright in portrait = roll about 90)
 - vert_ms2: up/down jolt = acceleration along gravity with gravity removed (works for any mounting)
Events CSV: timestamp_ms,lat,lon,speed_kmh,peak_ms2,p2p_ms2,kind,severity,pitch_deg,roll_deg
 - kind: pothole = drop first then impact, bump = rise first then drop (speed breaker)
 - severity: low / medium / high = peak jolt below 1.6x / 1.6-2.5x / above 2.5x the threshold
Settings in the app: jolt threshold (default 3.0 m/s2) and minimum speed (default 8 km/h, 0 = always on).
Detection is a heuristic: tune the threshold on your own drives.
