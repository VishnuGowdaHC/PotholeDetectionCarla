import carla
import pygame
from pygame.locals import *

class VehicleControl:
    def __init__(self, world, vehicle):
        self.vehicle = vehicle
        self.control = carla.VehicleControl()
        self._steer_cache = 0.0

    # ------------------------------------------------------------------ #
    #  WASD                                                                #
    # ------------------------------------------------------------------ #
    def _parse_vehicle_keys(self, keys, dt_ms):
        """Update throttle/brake/steer from WASD keys."""
        # Throttle
        if keys[K_w] or keys[K_UP]:
            self.control.throttle = min(self.control.throttle + 0.1, 1.0)
        else:
            self.control.throttle = 0.0

        # Brake
        if keys[K_s] or keys[K_DOWN]:
            self.control.brake = min(self.control.brake + 0.2, 1.0)
        else:
            self.control.brake = 0.0

        # Steer – gradual accumulation, snaps to 0 when key released
        steer_increment = 5e-4 * dt_ms
        if keys[K_a] or keys[K_LEFT]:
            self._steer_cache = 0 if self._steer_cache > 0 else self._steer_cache - steer_increment
        elif keys[K_d] or keys[K_RIGHT]:
            self._steer_cache = 0 if self._steer_cache < 0 else self._steer_cache + steer_increment
        else:
            self._steer_cache = 0.0

        self._steer_cache = max(-0.7, min(0.7, self._steer_cache))
        self.control.steer = round(self._steer_cache, 1)

        # Handbrake
        self.control.hand_brake = bool(keys[K_SPACE])

    # ------------------------------------------------------------------ #
    #  Main tick – call this every sync step                               #
    # ------------------------------------------------------------------ #
    def tick(self, clock):
        """
        Call once per simulation step (after world.tick() in sync mode).
        Returns True if the user wants to quit.
        """
        dt_ms = clock.get_time()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE or (event.key == K_q and pygame.key.get_mods() & KMOD_CTRL):
                    return True
                if event.key == K_q:                        # reverse
                    self.control.gear = 1 if self.control.reverse else -1
                    self.control.reverse = self.control.gear < 0

        keys = pygame.key.get_pressed()
        self._parse_vehicle_keys(keys, dt_ms)
        self.vehicle.apply_control(self.control)

        clock.tick(60)
        return False

    def destroy(self):
        pygame.quit()



