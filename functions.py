from pymavlink import mavutil
from collections import deque
from datetime import datetime
import paho.mqtt.client as mqtt
import json
import time
import math

MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 8884
MQTT_TOPIC = "luka/pixhawk"

def setup_mqtt():
    client = mqtt.Client(transport="websockets")
    client.tls_set()  # Enable SSL
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    return client

def publish_telemetry(client, data):
    """Publish data to MQTT"""
    client.publish(
        MQTT_TOPIC,
        payload=json.dumps(data),
        qos=0,
        retain=False
    )

# PX4 Flight Mode Definitions
PX4_MODES = {
    65536: "Manual",
    50593792: "Hold/Loiter",
    196608: "Position",
}

def connect_pixhawk(port='/dev/serial0', baud=57600):
    """Initialize MAVLink connection"""
    master = mavutil.mavlink_connection(port, baud=baud)
    master.wait_heartbeat()
    print(f"Heartbeat from system {master.target_system}")
    return master

def decode_flight_mode(msg):
    """Extract flight mode from HEARTBEAT message"""
    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
        return PX4_MODES.get(msg.custom_mode, f"UNKNOWN({msg.custom_mode})")
    return "UNKNOWN"

def get_current_mode(master):
    """Get the current flight mode"""
    msg = master.recv_match(type='HEARTBEAT', blocking=False)
    return decode_flight_mode(msg) if msg else None

def get_battery_status(master):
    """Request and return battery data"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
        0, 0, 0, 0, 0, 0
    )
    
    msg = master.recv_match(type='BATTERY_STATUS', blocking=True, timeout=1)
    if msg:
        return {
            'voltage': msg.voltages[0]/1000 if msg.voltages[0] != 65535 else 0,
            'current': msg.current_battery/100,
            'remaining': msg.battery_remaining if msg.battery_remaining != -1 else None
        }
    return None
    
def get_position(master):
    """Get GPS position and altitude"""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        return {
            'lat': msg.lat / 1e7,
            'lon': msg.lon / 1e7,
            'alt': msg.alt / 1000,  # meters
            'relative_alt': msg.relative_alt / 1000  # meters
        }
    return None
    
def get_attitude(master):
    """Get aircraft orientation"""
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    if msg:
        return {
            'roll': round(msg.roll, 2),
            'pitch': round(msg.pitch, 2),
            'yaw': round(msg.yaw, 2),
            'roll_deg': round(math.degrees(msg.roll), 2),
            'pitch_deg': round(math.degrees(msg.pitch), 2),
            'yaw_deg': round(math.degrees(msg.yaw), 2),
        }
    return None
    
def get_velocity(master):
    """Get 3D velocity vector"""
    msg = master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
    if msg:
        return {
            'groundspeed': msg.groundspeed,
            'airspeed': msg.airspeed,
            'climb_rate': msg.climb
        }
    return None
    
def get_gps_info(master):
    """Get detailed GPS status"""
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
    if msg:
        return {
            'fix_type': msg.fix_type,
            'satellites': msg.satellites_visible,
            'hdop': msg.eph / 100  # Horizontal dilution of precision
        }
    return None
    
def get_formatted_timestamp():
    now = datetime.now()
    return {
        "raw": time.time(),
        "time": now.strftime("%H:%M:%S"),
        "date": now.strftime("%Y-%m-%d")
    }
