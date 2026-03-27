import aria.sdk as aria
import os

print(f"--- Project Aria SDK Diagnostic v7 ---")

# Check StreamingClient
print(f"StreamingClient attributes: {dir(aria.StreamingClient)}")

# Check for observer methods
for name in ["set_streaming_client_observer", "subscribe", "unsubscribe"]:
    if hasattr(aria.StreamingClient, name):
        print(f"StreamingClient has {name}")
    else:
        print(f"StreamingClient does NOT have {name}")
