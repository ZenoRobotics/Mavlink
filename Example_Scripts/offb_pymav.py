import math
from pymavlink import mavutil
import time

# --- Connection Setup ---
# Replace 'udp:127.0.0.1:14550' with your drone's connection string.
# Common options include:
# - 'udp:127.0.0.1:14550' for SITL (Software In The Loop) simulation
# - '/dev/ttyACM0' for a serial connection (e.g., USB to Pixhawk)
# - 'tcp:192.168.1.10:5760' for TCP connection to a companion computer
#my_connection = mavutil.mavlink_connection('udpin:localhost:14540')
#my_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)



# --- Set offboard mode
def set_offboard_mode():
    print("Setting offboard mode")
    my_connection.mav.command_long_send(
       my_connection.target_system, my_connection.target_component,
       mavutil.mavlink.MAV_CMD_DO_SET_MODE,
       0,  # Confirmation
       base_mode,
       main_mode,
       sub_mode,
       0, 0, 0, 0)

    print("Sent MAV_CMD_DO_SET_MODE command for auto takeoff")
    ack("COMMAND_ACK")
    
#Class for formating the Mission Item.
class mission_item:
    def __init__(self, i, current, x,y,z):
       self.seq = i
       self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT  # Use Global latitude and Longitude for position data
       self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT         # Move to the waypoint
       self.current = current
       self.auto = 1
       self.param1 = 0.0
       self.param2 = 2.00
       self.param3 = 20.00
       self.param4 = math.nan
       self.param5 = x
       self.param6 = y
       self.param7 = z
       self.mission_type = 0 # The MAV_MISSION_TYPE value for MAV_MISSION_TYPE_MISSION

    
def upload_mission(mission_items):
    n = len(mission_items)
    print("Sending Message Out ... ")

    my_connection.mav.mission_count_send(my_connection.target_system, my_connection.target_component, 1, 0)

    ack("MISSION_REQUEST")
    waypoint = mission_items[0]
    my_connection.mav.mission_item_send(my_connection.target_system,          #Target System
                                 my_connection.target_component,       #Target Component
                                 waypoint.seq,                          #Sequence
                                 waypoint.frame,                        #Frame
                                 waypoint.command,                      #Command
                                 waypoint.current,                      #Current
                                 waypoint.auto,                         #Autocontinue
                                 waypoint.param1,                       #Hold Time
                                 waypoint.param2,                       #Accept Radius
                                 waypoint.param3,                       #Pass Radius (winged)
                                 waypoint.param4,                       #Yaw
                                 waypoint.param5,                       #Local X
                                 waypoint.param6,                       #Local Y
                                 waypoint.param7,                       #Local Z
                                 waypoint.mission_type)                 #Mission Type
    
    ack("MISSION_ACK")

    
    
# --- Arming ---
def arm():
    print("Arming motors...")
    # Request arming
    my_connection.mav.command_long_send(
        my_connection.target_system,
        my_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
        
    ack("COMMAND_ACK")
    print("Armed")

    # Wait for arming to complete
    my_connection.motors_armed_wait()
    #print("Motors armed!")
    time.sleep(1)

def takeoff(target_altitude):
    print(f"Taking off to {target_altitude} meters...")
    my_connection.mav.command_long_send(
        my_connection.target_system,
        my_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude)
        
    ack("COMMAND_ACK")
    print("Takeoff!")

    # Wait until target altitude is reached
    while True:
        msg = my_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0  # Convert mm to meters
        print(f"Current Altitude: {current_altitude:.2f}m")
        if current_altitude >= target_altitude * 0.95:  # Check if within 95% of target
            print("Target altitude reached!")
            break
        time.sleep(1)
        
#Send message for the drone to return to the launch point
def set_return():
    print("Set Return To Launch")
    my_connection.mav.command_long_send(the_connection.mav.command_long_send(
        my_connection.target_system,
        my_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0))
    ack("COMMAND_ACK")

# --- Landing ---
def land():
    print("Initiating landing...")
    my_connection.mav.command_long_send(
        my_connection.target_system,
        my_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0) # MAV_CMD_NAV_LAND with 0 altitude lands at current position

    # Wait for disarming (indicating landing complete)
    my_connection.motors_disarmed_wait()
    print("Motors disarmed. Landed!")
    
#Acknowledgement from the Drone
def ack(keyword):
    print("-- Message Read " + str(my_connection.recv_match(type=keyword, blocking=True)))


# --- Main Execution ---
try:
    # Define the custom mode parameters for PX4 (takeoff)
    base_mode = 128  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    main_mode = 4   # PX4_CUSTOM_MAIN_MODE_AUTO
    sub_mode = 1    # PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF
    
    my_connection = mavutil.mavlink_connection('udpin:localhost:14030')

    # Wait for the first heartbeat message to confirm connection
    my_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (my_connection.target_system, my_connection.target_component))
    
    target_altitude = 5
    
    mission_waypoints = []

    mission_waypoints.append(mission_item(0, 0, 45.66874762129226, -111.07818877582056, target_altitude))

    #upload_mission(mission_waypoints)
    #set_offboard_mode()  #not helping
    arm()  # Take off to 5 meters
    takeoff(target_altitude)  # Take off to 5 meters
    
    #for mission_item in mission_waypoints:
    #    print("-- Message Read " + str(my_connection.recv_match(type="MISSION_ITEM_REACHED",
    #          condition= 'MISSION_ITEM_REACHED.seq == {0}'.format(mission_item.seq), blocking=True)))
    
    #set_return()
    time.sleep(10)      # Hover for 10 seconds
    land()
except KeyboardInterrupt:
    print("Operation interrupted by user.")
finally:
    my_connection.close()
    print("Connection closed.")
    
