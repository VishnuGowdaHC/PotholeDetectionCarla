import carla
import math
import numpy as np
import pygame

class VehicleSensors():
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.z_acceleration = 9.81
        self.gs_data = None
        self.preview_surface = None
        self.sensors_list = []

        self.raw_preview = None
        self.raw_tof = None
        self.pothole_detected = False
        self.depression_depth = 0.0
        self.anomaly_frame_count = 0

    
    def _master_sensor_catcher(self, data, sensor_type):
        if sensor_type == "imu":
            self.z_acceleration = data.accelerometer.z
        elif sensor_type == "global_shutter":
            self.gs_data = data
        elif sensor_type == "preview":
            self.raw_preview = data
        elif sensor_type == "tof":
            self.raw_tof = data
        
        
    def setup(self):
        bp = self.world.get_blueprint_library()

        #IMU
        imu_bp = bp.find('sensor.other.imu')
        imu_loc = carla.Transform(carla.Location(x=0, y=0, z=2.5))
        imu = self.world.spawn_actor(imu_bp, imu_loc, attach_to=self.vehicle)
        self.sensors_list.append(imu)

        #tof
        tof_bp = bp.find('sensor.camera.depth')
        tof_bp.set_attribute('image_size_x', '400')
        tof_bp.set_attribute('image_size_y', '300')
        tof_bp.set_attribute('sensor_tick', '0.0')
        tof_bp.set_attribute('fov', '90')
        tof_loc = carla.Transform(carla.Location(x=2.5, y=0, z=1),
                                  carla.Rotation(pitch=-30, yaw=0))
        tof = self.world.spawn_actor(tof_bp, tof_loc, attach_to=self.vehicle)
        self.sensors_list.append(tof)
        
        #global_shutter
        global_shutter_bp = bp.find('sensor.camera.rgb')
        global_shutter_bp.set_attribute('image_size_x', '800')
        global_shutter_bp.set_attribute('image_size_y', '600')
        global_shutter_bp.set_attribute('blur_amount', '0.0')
        global_shutter_bp.set_attribute('sensor_tick', '0.0')
        global_shutter_loc = carla.Transform(
                                carla.Location(x=2.5, y=0, z=1),
                                carla.Rotation(pitch=-30, yaw=0))
        global_shutter = self.world.spawn_actor(global_shutter_bp, global_shutter_loc, attach_to=self.vehicle)
        self.sensors_list.append(global_shutter)

        #preview
        preview_bp = bp.find('sensor.camera.rgb')
        preview_bp.set_attribute('image_size_x', '800')
        preview_bp.set_attribute('image_size_y', '600')
        preview_bp.set_attribute('sensor_tick', '0.0')
        preview_loc = carla.Transform(carla.Location(x=-6, y=0, z=3),
                                      carla.Rotation(pitch=-15, yaw=0))
        preview = self.world.spawn_actor(preview_bp, preview_loc, attach_to=self.vehicle)
        self.sensors_list.append(preview)

        

        preview.listen(lambda data: self._master_sensor_catcher(data, "preview"))
        tof.listen(lambda data: self._master_sensor_catcher(data, "tof"))
        imu.listen(lambda data: self._master_sensor_catcher(data, "imu"))
        global_shutter.listen(lambda data: self._master_sensor_catcher(data, "global_shutter"))

        
    def destroy(self):
        for sensor in self.sensors_list:
            if sensor.is_alive:
                sensor.stop()
                sensor.destroy()

