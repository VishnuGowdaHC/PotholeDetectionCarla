import carla
import random

def scatter_props(world):
    blueprint_library = world.get_blueprint_library()
    prop_bp = blueprint_library.find('static.prop.dirtdebris03')

    waypoints = world.get_map().generate_waypoints(distance=30.0)

    random.shuffle(waypoints)

    spawned_count = 0

    for wp in waypoints[:10]:
        base_location = wp.transform.location
        offsets = [
            (-0.8, -0.8), (0.0, -0.8), (0.8, -0.8), # Back row
            (-0.8,  0.0), (0.0,  0.0), (0.8,  0.0), # Middle row
            (-0.8,  0.8), (0.0,  0.8), (0.8,  0.8)  # Front row
        ]

        for offset_x, offset_y in offsets:  
            spawn_loc = carla.Transform()
            spawn_loc.location.x = base_location.x + offset_x
            spawn_loc.location.y = base_location.y + offset_y

            spawn_loc.location.z = base_location.z + 0.35
            spawn_loc.rotation.pitch = 180.0

            try:
                prop = world.spawn_actor(prop_bp, spawn_loc)
                prop.set_simulate_physics(False)
                spawned_count += 1
            except:
                pass

    print(f"Spawned {spawned_count} props")
