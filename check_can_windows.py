import can
import time

def test_can():
    print('Searching for candleLight adapter...')
    try:
        # candleLight adapters on Windows often work through python-can with the gs_usb interface
        # or via a specific library. Let's try to list possible interfaces.
        bus = can.interface.Bus(interface='gs_usb', index=0, bitrate=1000000)
        print('Connected! Waiting for message...')
        msg = bus.recv(timeout=5)
        if msg:
            print(f'Received message: {msg}')
        else:
            print('Connected but no messages received within 5 seconds.')
            print('Is the robot powered on and sending data?')
        bus.shutdown()
    except Exception as e:
        print(f'Failed to connect: {e}')
        print('\nTroubleshooting:')
        print('1. Ensure you have detached the device from WSL (usbipd detach --busid 1-8)')
        print('2. If you still see errors, you might need SavvyCAN to verify drivers.')

if __name__ == '__main__':
    test_can()
