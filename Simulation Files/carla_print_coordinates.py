import carla
import time

client = carla.Client('localhost', 2000)
world = client.load_world('Town03')

print("Fly to your intersection. Printing coordinates...")
while True:
    transform = world.get_spectator().get_transform()
    print(f"x={transform.location.x:.2f}, y={transform.location.y:.2f}, z={transform.location.z:.2f} | yaw={transform.rotation.yaw:.2f}")
    time.sleep(1)