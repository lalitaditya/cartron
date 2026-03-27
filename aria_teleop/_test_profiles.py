import aria.sdk_gen2 as sdk_gen2
import time

def main():
    device_client = sdk_gen2.DeviceClient()
    config = sdk_gen2.DeviceClientConfig()
    # Using the hotspot IP from previous logs
    config.ip_v4_address = "172.20.10.5" 
    device_client.set_client_config(config)
    
    print("[test] Connecting to device...")
    device = device_client.connect()
    print("[test] Connected!")
    
    profiles = device.get_streaming_profiles()
    print(f"[test] Available profiles: {profiles}")
    
    # Try start streaming on one of the profiles if needed
    # (Just testing what's available for now)
    
    device_client.disconnect(device)
    print("[test] Disconnected.")

if __name__ == "__main__":
    main()
