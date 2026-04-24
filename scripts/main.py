import carla
import pygame
import numpy as np
from sensors import VehicleSensors
from keyboardControl import VehicleControl
from props import scatter_props


def main():
    pygame.init()
    display = pygame.display.set_mode((800, 600))
    font = pygame.font.Font(None, 25)

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    client.load_world('Town02')
    world = client.get_world()

    original_settings = world.get_settings()

    vehicle = None
    controller = None
    sensors = None

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.0166
        world.apply_settings(settings)

        blueprint_lib = world.get_blueprint_library()
        vehicle_bp = blueprint_lib.filter('vehicle.tesla.model3')[0]
        spawn_point = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)

        scatter_props(world)
        controller = VehicleControl(world, vehicle)
        sensors = VehicleSensors(world, vehicle)
        
        sensors.setup()
        clock = pygame.time.Clock()

        cooldown = 0
        EXPECTED_ROAD_DISTANCE = None

        while True:
            world.tick()
            if controller.tick(clock):
                break

            
            #tof
            if sensors.raw_tof is not None:
                image = sensors.raw_tof
                array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8")).reshape((image.height, image.width, 4))
                
                B = array[:, :, 0].astype(np.float32)
                G = array[:, :, 1].astype(np.float32)
                R = array[:, :, 2].astype(np.float32)
                depth_m = (R + G * 256.0 + B * 256.0 * 256.0) / (256.0**3 - 1) * 1000.0

                h, w = depth_m.shape
                roi = depth_m[int(h * 0.45):int(h * 0.55):3, int(w * 0.10):int(w * 0.90):3]

                baseline = np.median(roi, axis=1, keepdims=True)
                display_baseline = float(np.median(roi))
                                
                depression_map = roi - baseline
            
                DEPRESSION_THRESHOLD_M = 0.15  
                
                deepest_point = float(np.max(depression_map))

                # depressed_pixels = depression_map > DEPRESSION_THRESHOLD_M
                # area = int(np.sum(depressed_pixels))

                print(f"Road Baseline: {display_baseline:.2f}m | Deepest Hole: {deepest_point:.2f}m")
                if deepest_point > DEPRESSION_THRESHOLD_M:
                    sensors.anomaly_frame_count +=1

                    if sensors.anomaly_frame_count >= 2:
                        sensors.pothole_detected = True
                        sensors.depression_depth = deepest_point
                    
                else:
                    sensors.pothole_detected = False
                    sensors.depression_depth = 0.0
                    sensors.anomaly_frame_count = 0
            
            #trigger logic
            zImu = sensors.z_acceleration - 9.81

            tof_hit = sensors.pothole_detected
            imu_hit = zImu < -2.5

            if cooldown > 0:
                cooldown -= 1

            if (tof_hit or imu_hit) and cooldown == 0:
                if tof_hit and imu_hit:
                    print("Both sensors hit")
                elif tof_hit:
                    print("TOF sensor hit")
                    if sensors.gs_data is not None:
                        sensors.gs_data.save_to_disk(f'_out/anomaly_{sensors.gs_data.frame}.png')
                else:
                    print("IMU sensor hit")      

                cooldown = 60
            
            if sensors.raw_preview is not None:
                p_image = sensors.raw_preview
                p_array = np.frombuffer(p_image.raw_data, dtype=np.dtype("uint8"))
                p_array = np.reshape(p_array, (p_image.height, p_image.width, 4))
                p_array = p_array[:, :, :3]
                p_array = p_array[:, :, ::-1]
                surface = pygame.surfarray.make_surface(p_array.swapaxes(0,1))

                if surface.get_size() != display.get_size():
                    surface = pygame.transform.scale(surface, display.get_size())

                display.blit(surface, (0, 0))

                color = (255, 50, 50) if tof_hit else (50, 255, 50)
                
                depth_text = font.render(f"Depth: {sensors.depression_depth:.2f} m", True, color)
                zImu_text = font.render(f"Z Imu: {zImu:.2f}m/s^2", True, (255, 255, 50))

                display.blit(depth_text, (10, 10))
                display.blit(zImu_text, (10, 40))
                
                pygame.display.flip()
           
    finally:
        world.apply_settings(original_settings)
        if sensors:
            sensors.destroy()
        
        if controller:
            controller.destroy()

        if vehicle:
            vehicle.destroy()

        pygame.quit()

if __name__ == '__main__':
    main()
    