import serial.tools.list_ports

def find_candlelight():
    print("Searching for candleLight adapter (1D50:606F)...")
    ports = serial.tools.list_ports.comports()
    found = False
    for port in ports:
        print(f"Checking {port.device}: {port.hwid}")
        if "1D50:606F" in port.hwid.upper():
            print(f"*** FOUND CANDLELIGHT ON {port.device} ***")
            found = True
    if not found:
        print("CandleLight adapter NOT found in COM port list.")
        print("Is it in the Device Manager as 'Other Devices' or 'WinUSB'?")

if __name__ == "__main__":
    find_candlelight()
