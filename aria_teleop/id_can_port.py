import serial.tools.list_ports

def scan_ports():
    print("Listing all available COM ports and descriptions:")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"Port: {port.device}")
        print(f"  Description: {port.description}")
        print(f"  HWID: {port.hwid}")
        print("-" * 20)

if __name__ == "__main__":
    scan_ports()
