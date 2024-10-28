from pymavlink import mavutil
import time

# test wp
WAYPOINT_LAT = 38.870972   # 38°52'15.5"N in decimal degrees
WAYPOINT_LON = -77.322194  # 77°19'19.9"W in decimal degrees
WAYPOINT_ALT = 30.48       # 100 ft in meters

def connect_to_mavlink():
    # tcp connection via port 14551 localhost
    mavlink_connection = mavutil.mavlink_connection("tcp:localhost:14551")
    # heartbeat
    mavlink_connection.wait_heartbeat()
    print("Connected to MAVLink!")
    return mavlink_connection

def send_waypoint(mavlink_connection, lat, lon, alt):
    # send wp defiend previosuly here
    mavlink_connection.mav.set_position_target_global_int_send(
        0,                      # time boot ms (unused)
        mavlink_connection.target_system,  # target system
        mavlink_connection.target_component,  # target components
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # set relative alt
        int(0b110111111000),    # mask hex
        int(lat * 1e7),         # lat in integer
        int(lon * 1e7),         # long in integer
        alt,                    # altitude in meters (relative to takeoff point)
        0, 0, 0,                # velocity in m/s
        0, 0, 0,                # accelerations
        0, 0                    # yaw
    )
    print(f"Waypoint set to {lat}, {lon} at {alt} meters.")

def send_land_command(mavlink_connection):
    # land command
    mavlink_connection.mav.command_long_send(
        mavlink_connection.target_system,
        mavlink_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,                       # confirm
        0, 0,                    # params 1-2 (unused)
        0, 0,                    # params 3-4 (unused)
        0, 0, 0                  # latitude, longitude, altitude (use current)
    )
    print("Land command sent.")

def main():
    mavlink_connection = connect_to_mavlink()
    
    print("Press any key to go to waypoint...")
    input()

    # senmd wp command
    send_waypoint(mavlink_connection, WAYPOINT_LAT, WAYPOINT_LON, WAYPOINT_ALT)
    
    # pause to wait for drone to get to wp
    time.sleep(10)

    # send land command at wp
    send_land_command(mavlink_connection)

if __name__ == "__main__":
    main()
