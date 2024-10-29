import cv2
import numpy as np
from pymavlink import mavutil
import time
import threading
import math
import queue

# Camera setup
camera_id = int(input("Enter camera ID: "))
window = "Camera View"
cv2.namedWindow(window)

# ArUco marker setup
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters()
arucoDetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

# Global queues and locks
message_queue = queue.Queue()
command_lock = threading.Lock()

# Active tasks
active_tasks = {}  # Keep track of active tasks per ArUco ID

# MAVLink connection setup
def connect_to_mavlink():
    mavlink_connection = mavutil.mavlink_connection("tcp:192.168.1.193:14551")
    mavlink_connection.wait_heartbeat()
    print("Connected to MAVLink!")
    return mavlink_connection

# MAVLink listener thread
def mavlink_listener(mavlink_connection):
    while True:
        msg = mavlink_connection.recv_match(blocking=True)
        if msg:
            message_queue.put(msg)

# Set mode function
def set_mode(mavlink_connection, mode):
    with command_lock:
        if mode not in mavlink_connection.mode_mapping():
            print(f"Mode {mode} is not available.")
            return False

        mode_id = mavlink_connection.mode_mapping()[mode]
        mavlink_connection.mav.set_mode_send(
            mavlink_connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

    # Wait for ACK
    while True:
        try:
            msg = message_queue.get(timeout=1)
            if msg.get_type() == 'HEARTBEAT':
                if msg.custom_mode == mode_id:
                    print(f"Mode changed to {mode}")
                    break
        except queue.Empty:
            continue
    return True

# Send waypoint command
def send_waypoint(mavlink_connection, lat, lon, alt):
    with command_lock:
        # Define the type_mask to ignore velocities, accelerations, yaw, and yaw rate
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        mavlink_connection.mav.set_position_target_global_int_send(
            0,
            mavlink_connection.target_system,
            mavlink_connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,  # Velocities (ignored)
            0, 0, 0,  # Accelerations (ignored)
            0, 0      # Yaw, Yaw rate (ignored)
        )
    print(f"Waypoint set to {lat}, {lon} at {alt} meters.")

# Landing command
def send_land_command(mavlink_connection):
    with command_lock:
        mavlink_connection.mav.command_long_send(
            mavlink_connection.target_system,
            mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0,
            0, 0,
            0, 0, 0
        )
    print("Land command sent.")

# Arm the drone
def arm_drone(mavlink_connection):
    with command_lock:
        mavlink_connection.mav.command_long_send(
            mavlink_connection.target_system,
            mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # Arm (1 to arm, 0 to disarm)
            0, 0, 0, 0, 0, 0
        )
    print("Drone armed.")

# Takeoff command with altitude confirmation
def takeoff_drone(mavlink_connection, altitude):
    with command_lock:
        mavlink_connection.mav.command_long_send(
            mavlink_connection.target_system,
            mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0, altitude
        )
    print("Takeoff initiated.")

    # Confirm altitude increase to ensure takeoff
    while True:
        try:
            msg = message_queue.get(timeout=1)
            if msg.get_type() == "GLOBAL_POSITION_INT":
                if msg.relative_alt >= altitude * 1000 * 0.9:  # Check if within 90% of target altitude
                    print(f"Reached takeoff altitude of {altitude} meters")
                    break
        except queue.Empty:
            continue
    # Set mode to GUIDED after reaching altitude
    set_mode(mavlink_connection, "GUIDED")
    time.sleep(2)  # Allow time for mode change

# Return to launch command
def rtl(mavlink_connection):
    with command_lock:
        mavlink_connection.mav.command_long_send(
            mavlink_connection.target_system,
            mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
    print("Returning to Launch (RTL)")
    # Optional: Remove from active_tasks if needed

# Waypoints based on ArUco tag IDs
waypoints = {
    1: {"lat": xx, "lon": xx, "alt": 30.48},
    2: {"lat": xx, "lon": xx, "alt": 30.48}
}

# Processed ArUco tag IDs
processed_ids = set()

# Calculate distance between two GPS coordinates
def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371  # Earth radius in km
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c * 1000  # Return distance in meters

# Waiting until arrival at waypoint
def wait_until_arrival(target_lat, target_lon, threshold=2):
    while True:
        try:
            msg = message_queue.get(timeout=1)
            if msg.get_type() == "GLOBAL_POSITION_INT":
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7
                distance = calculate_distance(current_lat, current_lon, target_lat, target_lon)
                print(f"Distance to waypoint: {distance:.2f} meters (Current position: {current_lat}, {current_lon})")
                if distance < threshold:
                    print("Arrival confirmed!")
                    break
        except queue.Empty:
            continue

# Wait until drone is disarmed
def wait_until_disarmed():
    while True:
        try:
            msg = message_queue.get(timeout=1)
            if msg.get_type() == "HEARTBEAT":
                if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    print("Drone disarmed after landing.")
                    break
        except queue.Empty:
            continue

# Navigate to waypoint
def navigate_to_waypoint(mavlink_connection, aruco_id):
    global active_tasks
    wp = waypoints[aruco_id]

    # First navigation sequence
    if set_mode(mavlink_connection, "GUIDED"):
        print("Mode set to GUIDED.")
        arm_drone(mavlink_connection)
        time.sleep(2)
        takeoff_drone(mavlink_connection, wp["alt"])
        send_waypoint(mavlink_connection, wp["lat"], wp["lon"], wp["alt"])
        wait_until_arrival(wp["lat"], wp["lon"])
        send_land_command(mavlink_connection)
        wait_until_disarmed()

    # Prepare for the second navigation sequence
    if set_mode(mavlink_connection, "GUIDED"):
        print("Mode set to GUIDED for second sequence.")
        arm_drone(mavlink_connection)
        time.sleep(2)
        takeoff_drone(mavlink_connection, wp["alt"])
        send_waypoint(mavlink_connection, wp["lat"], wp["lon"], wp["alt"])
        wait_until_arrival(wp["lat"], wp["lon"])
        send_land_command(mavlink_connection)
        wait_until_disarmed()

    # Remove from active_tasks
    active_tasks.pop(aruco_id, None)

# Main function
def main():
    mavlink_connection = connect_to_mavlink()
    threading.Thread(target=mavlink_listener, args=(mavlink_connection,), daemon=True).start()
    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        corners, ids, _ = arucoDetector.detectMarkers(frame)

        if ids is not None:
            for i in range(len(ids)):
                aruco_id = ids[i][0]
                cv2.aruco.drawDetectedMarkers(frame, [corners[i]], ids[i])
                corner = corners[i][0][0]
                cv2.putText(frame, f"ID: {aruco_id}", (int(corner[0]), int(corner[1] - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if aruco_id not in processed_ids and aruco_id not in active_tasks:
                    processed_ids.add(aruco_id)
                    if aruco_id in waypoints:
                        task_thread = threading.Thread(target=navigate_to_waypoint, args=(mavlink_connection, aruco_id))
                        active_tasks[aruco_id] = task_thread
                        task_thread.start()
                    elif aruco_id == 3:
                        task_thread = threading.Thread(target=rtl, args=(mavlink_connection,))
                        active_tasks[aruco_id] = task_thread
                        task_thread.start()

        y_position = 30
        cv2.putText(frame, "Tags read:", (frame.shape[1] - 150, y_position), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        for tag_id in processed_ids:
            y_position += 20
            cv2.putText(frame, f"Tag {tag_id}", (frame.shape[1] - 150, y_position), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow(window, frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
