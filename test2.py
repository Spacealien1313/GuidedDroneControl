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

# Global status variables
drone_status = "Idle"
current_altitude = None
current_latitude = None
current_longitude = None
drone_armed = None

# MAVLink connection setup
def connect_to_mavlink():
    mavlink_connection = mavutil.mavlink_connection("tcp:192.168.1.193:14551")
    mavlink_connection.wait_heartbeat()
    print("Connected to MAVLink!")
    return mavlink_connection

# MAVLink listener thread
def mavlink_listener(mavlink_connection):
    global current_altitude, current_latitude, current_longitude, drone_armed
    while True:
        msg = mavlink_connection.recv_match(blocking=True)
        if msg:
            msg_type = msg.get_type()
            # Process messages
            if msg_type == 'COMMAND_ACK':
                print(f"Command Acknowledged: Command={msg.command}, Result={msg.result}")
            elif msg_type == 'STATUSTEXT':
                print(f"Status Message: {msg.text}")
            elif msg_type == 'GLOBAL_POSITION_INT':
                current_altitude = msg.relative_alt / 1000.0  # Convert mm to meters
                current_latitude = msg.lat / 1e7
                current_longitude = msg.lon / 1e7
            elif msg_type == 'HEARTBEAT':
                drone_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            # Put the message in the queue for other threads
            message_queue.put(msg)

# Function to report status every 10 seconds
def status_reporter():
    global drone_status
    while True:
        print(f"Status Update: {drone_status}")
        time.sleep(10)

# Request data stream from the drone
def request_data_stream(mavlink_connection):
    mavlink_connection.mav.request_data_stream_send(
        mavlink_connection.target_system,
        mavlink_connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,  # Frequency (Hz)
        1   # Start sending (0 to stop)
    )

# Set mode function
def set_mode(mavlink_connection, mode):
    with command_lock:
        if mode not in mavlink_connection.mode_mapping():
            print(f"Mode {mode} is not available.")
            print("Available modes:", mavlink_connection.mode_mapping())
            return False

        mode_id = mavlink_connection.mode_mapping()[mode]
        mavlink_connection.mav.set_mode_send(
            mavlink_connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
    print(f"Mode change to {mode} requested.")

    # Wait for mode change confirmation
    start_time = time.time()
    timeout = 10  # seconds
    while True:
        if time.time() - start_time > timeout:
            print(f"Timeout while waiting for mode change to {mode}.")
            return False
        try:
            msg = message_queue.get(timeout=1)
            if msg.get_type() == 'HEARTBEAT':
                if msg.custom_mode == mode_id:
                    print(f"Mode changed to {mode}")
                    break
        except queue.Empty:
            continue
    return True

# Arm the drone with command acknowledgment
def arm_drone(mavlink_connection):
    with command_lock:
        mavlink_connection.mav.command_long_send(
            mavlink_connection.target_system,
            mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # Arm
            0, 0, 0, 0, 0, 0
        )
    print("Arm command sent.")

    # Wait for acknowledgment
    start_time = time.time()
    timeout = 10  # seconds
    while True:
        if time.time() - start_time > timeout:
            print("Timeout while waiting for arm acknowledgment.")
            return
        try:
            msg = message_queue.get(timeout=1)
            if msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if msg.result == 0:
                    print("Drone armed successfully.")
                else:
                    print(f"Failed to arm drone: result={msg.result}")
                break
        except queue.Empty:
            continue

# Takeoff command with corrected parameters
def takeoff_drone(mavlink_connection, altitude):
    with command_lock:
        print("Sending takeoff command...")
        mavlink_connection.mav.command_long_send(
            mavlink_connection.target_system,
            mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            float('nan'), float('nan'), altitude  # Set latitude and longitude to NaN
        )
    print("Takeoff command sent.")

    # Wait for acknowledgment
    start_time = time.time()
    timeout = 10  # seconds
    while True:
        if time.time() - start_time > timeout:
            print("Timeout while waiting for takeoff acknowledgment.")
            return
        try:
            msg = message_queue.get(timeout=1)
            if msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                if msg.result == 0:
                    print("Takeoff command acknowledged.")
                else:
                    print(f"Failed to initiate takeoff: result={msg.result}")
                break
        except queue.Empty:
            continue

    # Confirm altitude increase to ensure takeoff
    start_time = time.time()
    timeout = 60  # seconds
    while True:
        if time.time() - start_time > timeout:
            print("Takeoff timeout reached.")
            return
        if current_altitude is not None:
            if current_altitude >= altitude * 0.9:  # Check if within 90% of target altitude
                print(f"Reached takeoff altitude of {altitude} meters")
                break
        else:
            time.sleep(0.1)
            continue

# Return to Launch (RTL) command
def rtl(mavlink_connection):
    with command_lock:
        mavlink_connection.mav.command_long_send(
            mavlink_connection.target_system,
            mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
    print("Return to Launch (RTL) command sent.")

    # Wait for acknowledgment
    start_time = time.time()
    timeout = 10  # seconds
    while True:
        if time.time() - start_time > timeout:
            print("Timeout while waiting for RTL acknowledgment.")
            return
        try:
            msg = message_queue.get(timeout=1)
            if msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                if msg.result == 0:
                    print("RTL command acknowledged.")
                else:
                    print(f"Failed to initiate RTL: result={msg.result}")
                break
        except queue.Empty:
            continue


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

# Wait until arrival at waypoint
def wait_until_arrival(target_lat, target_lon, threshold=2):
    start_time = time.time()
    timeout = 120  # seconds
    while True:
        if time.time() - start_time > timeout:
            print("Timeout reached while waiting to arrive at waypoint.")
            break
        if current_latitude is not None and current_longitude is not None:
            current_lat = current_latitude
            current_lon = current_longitude
            distance = calculate_distance(current_lat, current_lon, target_lat, target_lon)
            if distance < threshold:
                print("Arrival at waypoint confirmed!")
                break
        else:
            time.sleep(0.1)
            continue

# Landing command with acknowledgment
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

    # Wait for acknowledgment
    start_time = time.time()
    timeout = 10  # seconds
    while True:
        if time.time() - start_time > timeout:
            print("Timeout while waiting for land acknowledgment.")
            return
        try:
            msg = message_queue.get(timeout=1)
            if msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                if msg.result == 0:
                    print("Land command acknowledged.")
                else:
                    print(f"Failed to initiate landing: result={msg.result}")
                break
        except queue.Empty:
            continue

# Wait until drone is disarmed
def wait_until_disarmed():
    start_time = time.time()
    timeout = 60  # seconds
    while True:
        if time.time() - start_time > timeout:
            print("Timeout reached while waiting for drone to disarm.")
            break
        if drone_armed is not None:
            if not drone_armed:
                print("Drone disarmed after landing.")
                break
        else:
            time.sleep(0.1)
            continue

# Calculate distance between two GPS coordinates
def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371  # Earth radius in km
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c * 1000  # Return distance in meters

# Waypoints based on ArUco tag IDs
waypoints = {
    1: {"lat": 38.871139, "lon": -77.321917, "alt": 30.48},  # 100 feet in meters
    2: {"lat": 38.870667, "lon": -77.322444, "alt": 30.48}
}

# Processed ArUco tag IDs
processed_ids = set()

# Function to check if the drone is airborne
def is_drone_airborne():
    global current_altitude, drone_armed
    # Check if the drone is airborne based on altitude and armed status
    start_time = time.time()
    timeout = 5  # seconds
    while True:
        if time.time() - start_time > timeout:
            print("Timeout reached while checking if drone is airborne.")
            return False  # Default to False if unable to determine
        if drone_armed is not None:
            if drone_armed and current_altitude is not None:
                if current_altitude > 1.0:
                    return True
                else:
                    return False
            else:
                return False
        else:
            time.sleep(0.1)
            continue

# Navigate to waypoint
def navigate_to_waypoint(mavlink_connection, aruco_id):
    global active_tasks, drone_status
    wp = waypoints[aruco_id]

    def navigation_sequence():
        global drone_status
        print(f"Starting navigation sequence for ArUco ID: {aruco_id}")
        # Check if drone is airborne
        airborne = is_drone_airborne()
        if not airborne:
            # Drone is not airborne, proceed to set mode to GUIDED
            if set_mode(mavlink_connection, "GUIDED"):
                print("Mode set to GUIDED.")
                drone_status = "Mode set to GUIDED"
                # Wait 2 seconds before arming
                time.sleep(2)
                # Arm the drone
                arm_drone(mavlink_connection)
                drone_status = "Drone armed, preparing to take off"
                takeoff_drone(mavlink_connection, wp["alt"])
                drone_status = "Taking off"
                # Check if takeoff was successful
                if not is_drone_airborne():
                    print("Takeoff failed, aborting navigation sequence.")
                    drone_status = "Takeoff failed"
                    return
            else:
                print("Failed to set mode to GUIDED at the beginning.")
                drone_status = "Failed to set mode to GUIDED"
                return
        else:
            print("Drone is already airborne, skipping arm and takeoff.")
            drone_status = "Drone already airborne"

        # Proceed with navigation
        drone_status = "Navigating to waypoint"
        send_waypoint(mavlink_connection, wp["lat"], wp["lon"], wp["alt"])
        wait_until_arrival(wp["lat"], wp["lon"])
        drone_status = "Landing"
        send_land_command(mavlink_connection)
        wait_until_disarmed()
        drone_status = "Landed"

        # Add delay after disarming
        time.sleep(2)  # Delay before attempting to set mode to GUIDED and arm

        # After landing and disarming, proceed to set mode to GUIDED and take off again
        if set_mode(mavlink_connection, "GUIDED"):
            print("Mode set to GUIDED after landing.")
            drone_status = "Mode set to GUIDED after landing"
            # Wait 2 seconds after mode change
            time.sleep(2)
            # Arm the drone
            arm_drone(mavlink_connection)
            drone_status = "Drone armed, preparing to take off again"
            takeoff_drone(mavlink_connection, 30.48)  # 100 feet in meters
            drone_status = "Ascending to 100 ft"
            
            # Stay in GUIDED mode after reaching altitude
            print("Drone will remain in GUIDED mode after takeoff.")
            drone_status = "Hovering in GUIDED mode"
        else:
            print("Failed to set mode to GUIDED after landing.")
            drone_status = "Failed to set mode to GUIDED after landing"

        # Remove from active_tasks
        active_tasks.pop(aruco_id, None)

    # Start the navigation sequence in a new thread to avoid blocking
    task_thread = threading.Thread(target=navigation_sequence, daemon=True)
    active_tasks[aruco_id] = task_thread
    task_thread.start()

# Main function
def main():
    global drone_status
    mavlink_connection = connect_to_mavlink()
    request_data_stream(mavlink_connection)
    threading.Thread(target=mavlink_listener, args=(mavlink_connection,), daemon=True).start()
    threading.Thread(target=status_reporter, daemon=True).start()  # Start the status reporter thread
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
            print(f"Detected ArUco IDs: {ids.flatten()}")
            for i in range(len(ids)):
                aruco_id = ids[i][0]
                cv2.aruco.drawDetectedMarkers(frame, [corners[i]], ids[i])
                corner = corners[i][0][0]
                cv2.putText(frame, f"ID: {aruco_id}", (int(corner[0]), int(corner[1] - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if aruco_id not in processed_ids and aruco_id not in active_tasks:
                    processed_ids.add(aruco_id)
                    print(f"Processing ArUco ID: {aruco_id}")
                    if aruco_id in waypoints:
                        navigate_to_waypoint(mavlink_connection, aruco_id)
                    elif aruco_id == 3:
                        threading.Thread(target=rtl, args=(mavlink_connection,), daemon=True).start()
                else:
                    print(f"ArUco ID: {aruco_id} already processed or active.")

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
