import carla

client = carla.Client('localhost', 2000)
# Reloading the world guarantees a clean slate for each run
world = client.load_world('Town03') 

# 1. Select the vehicle blueprint
bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.find('vehicle.tesla.model3')
vehicle_bp.set_attribute('role_name', 'ego')

# 2. Hardcode your extracted transform
# Replace these with your copied values, adding +1.0 to Z
spawn_point = carla.Transform(
    carla.Location(x=-6, y=-77.7, z=0.5 + 1.0), 
    carla.Rotation(pitch=0.0, yaw=270.0, roll=0.0)
)

# 3. Spawn the vehicle
ego_vehicle = world.spawn_actor(vehicle_bp, spawn_point)
print("Vehicle spawned successfully at the fixed intersection!")


settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05 # Locks the simulation to 20 FPS
world.apply_settings(settings)

# Note: Because you are now in synchronous mode, 
# you must command the server to step forward in your main loop:
# world.tick()