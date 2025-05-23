from functions import *
import time

master = connect_pixhawk()
mqtt_client = setup_mqtt()
mqtt_client.loop_start()  # Start MQTT background thread
last_mode = None

while True:
    # GETS THE CURRENT FLIGHT MODE WHEN IT CHANGES
    current_mode = get_current_mode(master)
    if current_mode and current_mode != last_mode:
        print(f"\nFLIGHT MODE CHANGED: {current_mode}")
        last_mode = current_mode
    
    # GETS THE BATTERY VOLTS, AMPS AND REMAINING PERSENTAGE
    #battery = get_battery_status(master)
    #if battery:
    #print(f"\rBattery: {battery['voltage']:.2f}V | {battery['remaining']}%", end='')
    
    #pos = get_position(master)
    #if pos:
        #print(f"\nPOS: Lat={pos['lat']:.6f}, Lon={pos['lon']:.6f}")
        #print(f"ALT: {pos['alt']:.1f}m (Rel: {pos['relative_alt']:.1f}m)")
    
    # GETS THE ROLL, PITCH AND YAW
    #att = get_attitude(master)
    #if att:
        #print(f"\rATT: Roll={math.degrees(att['roll']):.1f}° | "
              #f"Pitch={math.degrees(att['pitch']):.1f}° | "
              #f"Yaw={math.degrees(att['yaw']):.1f}°", end='')

    # GETS THE GROUNDSPEED, AIRSPEED AND CLIMB RATE
    #vel = get_velocity(master)
    #if vel:
        #print(f"\rVEL: Groundspeed={vel['groundspeed']:.1f}° | "
              #f"Airspeed={vel['airspeed']:.1f}° | "
              #f"Climb Rate={vel['climb_rate']:.1f}°", end='')
              
    #gps = get_gps_info(master)
    #if gps:
        #print(f"\rGPS: Fix Type={gps['fix_type']:.1f}° | "
              #f"Satellites={gps['satellites']:.1f}° | "
              #f"Hdop={gps['hdop']:.1f}°", end='')
              
    # Get and publish telemetry
    data = {
        'battery': get_battery_status(master),
        'position': get_position(master),
        'attitude': get_attitude(master),
        'mode': last_mode,
        'timestamp': get_formatted_timestamp()
    }
    
    publish_telemetry(mqtt_client, data)
    
    
    
    time.sleep(1)  # Small delay to prevent CPU overload
    
mqtt_client.loop_stop()
