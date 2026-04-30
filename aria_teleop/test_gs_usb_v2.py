import can
from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
import usb.core
import libusb_package

def test_gs_usb_direct():
    print("Initializing libusb backend...")
    import usb.backend.libusb1
    # Find the libusb-1.0 backend from libusb-package
    try:
        backend = usb.backend.libusb1.get_backend(find_library=libusb_package.get_library_path)
        if backend is None:
            print("Failed to load libusb backend.")
            return
    except Exception as e:
        print(f"Backend Init Error: {e}")
        return
    
    print("Searching for candleLight (1D50:606F)...")
    dev = usb.core.find(idVendor=0x1d50, idProduct=0x606f, backend=backend)
    if dev:
        print(f"SUCCESS: Found candleLight hardware: {dev.product}")
    else:
        print("CandleLight NOT found. Check physical connection.")
        return

    print("Listing GS_USB devices...")
    devices = GsUsb.scan()
    if not devices:
        print("No GS_USB devices found. Check Zadig/WinUSB driver.")
        return

    for i, dev in enumerate(devices):
        print(f"Device {i}: {dev}")
    
    print("\nAttempting to connect via python-can...")
    try:
        # Modern python-can gs_usb connection
        bus = can.Bus(interface='gs_usb', channel=devices[0].bus, bus=devices[0].address, bitrate=1000000)
        print("SUCCESS: Connected to GS_USB via python-can!")
        msg = bus.recv(timeout=2.0)
        if msg:
            print(f"Data received: {msg}")
        else:
            print("Connected, but no messages (Robot heartbeat 0x2A1 not seen).")
        bus.shutdown()
    except Exception as e:
        print(f"python-can connection failed: {e}")
        print("Falling back to raw gs_usb test...")
        try:
            dev = devices[0]
            dev.set_bitrate(1000000)
            dev.start()
            print("SUCCESS: Raw GS_USB started!")
            dev.stop()
        except Exception as e2:
            print(f"Raw GS_USB failed: {e2}")

if __name__ == "__main__":
    test_gs_usb_direct()
