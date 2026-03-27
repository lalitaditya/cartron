"""
Aria 3D Skeleton Viewer (Windows)
=================================
Listens for hand tracking data on UDP port 5010 and displays a 3D skeleton.
Works with both aria_udp_streamer.py (from WSL) and aria_piper_bridge.py --sim.

Requirements:
  pip install matplotlib numpy
"""
import json
import socket
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Aria Gen 2 Landmark Indices
# WRIST=5, THUMB_TIP=0, INDEX_TIP=1, MIDDLE_TIP=2, RING_TIP=3, PINKY_TIP=4
SKELETON_CONNECTIONS = [
    [5, 6, 7, 0],       # Thumb
    [5, 8, 9, 10, 1],   # Index
    [5, 11, 12, 13, 2], # Middle
    [5, 14, 15, 16, 3], # Ring
    [5, 17, 18, 19, 4], # Pinky
]

class AriaVizViewer:
    def __init__(self, port=5010):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.setblocking(False)
        self.latest_landmarks = None
        self.handedness = "Unknown"
        self.confidence = 0.0

        # Set up plot
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title("Aria Hand Skeleton (3D)")
        
        # Scaling (Aria device frame is meters)
        self.ax.set_xlim3d([-0.2, 0.2])
        self.ax.set_ylim3d([-0.2, 0.2])
        self.ax.set_zlim3d([0.1, 0.6])
        self.ax.set_xlabel("X (Right)")
        self.ax.set_ylabel("Y (Down)")
        self.ax.set_zlabel("Z (Forward)")

        # Persistent plot elements
        self.scatter = self.ax.scatter([], [], [], c='r', s=20)
        self.lines = [self.ax.plot([], [], [], '-', lw=2)[0] for _ in SKELETON_CONNECTIONS]
        self.text = self.ax.text2D(0.05, 0.95, "", transform=self.ax.transAxes)

    def update_data(self):
        """Read all available UDP packets and keep the latest."""
        while True:
            try:
                data, addr = self.sock.recvfrom(65535)
                packet = json.loads(data.decode('utf-8'))
                if packet.get("type") == "hand_tracking" and "landmarks" in packet:
                    self.latest_landmarks = np.array(packet["landmarks"])
                    self.handedness = packet.get("handedness", "Unknown")
                    self.confidence = packet.get("confidence", 0.0)
            except (socket.error, json.JSONDecodeError):
                break

    def animate(self, i):
        self.update_data()
        
        # Periodic heartbeat log
        if i % 30 == 0:
            status = "RECEIVING DATA" if self.latest_landmarks is not None else "WAITING FOR DATA"
            print(f"\r[viewer] {status} | port {self.sock.getsockname()[1]} ", end="")

        if self.latest_landmarks is None:
            return self.scatter, *self.lines, self.text

        # Update points
        lms = self.latest_landmarks
        self.scatter._offsets3d = (lms[:, 0], lms[:, 1], lms[:, 2])

        # Update skeleton lines
        for line, conn in zip(self.lines, SKELETON_CONNECTIONS):
            pts = lms[conn]
            line.set_data(pts[:, 0], pts[:, 1])
            line.set_3d_properties(pts[:, 2])

        # Update text
        self.text.set_text(f"Hand: {self.handedness}\nConf: {self.confidence:.2f}")
        
        return self.scatter, *self.lines, self.text

    def run(self):
        print(f"Listening for hand data on UDP port 5010...")
        print("Rotate/Zoom the 3D plot with your mouse.")
        # Fix for Matplotlib animation warning and Tkinter stability
        ani = animation.FuncAnimation(
            self.fig, self.animate, interval=33, 
            blit=False, cache_frame_data=False, save_count=100
        )
        plt.show()

if __name__ == "__main__":
    viewer = AriaVizViewer()
    viewer.run()
