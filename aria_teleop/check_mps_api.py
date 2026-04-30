import aria.sdk_gen2 as sdk_gen2

print("=== StreamingInterface options ===")
for name in dir(sdk_gen2.StreamingInterface):
    if not name.startswith("_"):
        print(f"  {name}")

print("\n=== HttpStreamingConfig attrs ===")
cfg = sdk_gen2.HttpStreamingConfig()
for name in dir(cfg):
    if not name.startswith("_"):
        print(f"  {name} = {getattr(cfg, name, '?')}")
