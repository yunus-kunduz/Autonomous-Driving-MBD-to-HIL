import carla
import time
import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0.0
        self.sum_error = 0.0

    def step(self, error, dt):
        self.sum_error += error * dt
        derivative = (error - self.last_error) / dt
        output = (self.Kp * error) + (self.Ki * self.sum_error) + (self.Kd * derivative)
        self.last_error = error
        return output

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    spectator = world.get_spectator()
    
    actor_list = []

    try:
        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.find('vehicle.tesla.model3')
        if bp.has_attribute('color'):
            bp.set_attribute('color', '0,0,255')

        # DÜZELTME: Merdivenlerde doğmamak için rastgele güvenli bir nokta seçiyoruz
        spawn_points = world.get_map().get_spawn_points()
        spawn_point = random.choice(spawn_points) 
        
        vehicle = world.spawn_actor(bp, spawn_point)
        actor_list.append(vehicle)

        speed_controller = PIDController(Kp=0.8, Ki=0.1, Kd=0.02)
        target_speed = 30.0 

        while True:
            # --- YENİ KAMERA AÇISI (Üçüncü Şahıs Görünümü) ---
            v_transform = vehicle.get_transform()
            # Aracın 10 metre arkasında, 5 metre yukarısında ve yola bakan bir açı
            # Not: Rotation(pitch=-15) yolu görmeni sağlar
            spectator.set_transform(carla.Transform(
                v_transform.location + v_transform.get_forward_vector() * -10 + carla.Location(z=5),
                carla.Rotation(pitch=-15, yaw=v_transform.rotation.yaw)
            ))

            velocity = vehicle.get_velocity()
            current_speed = 3.6 * np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            
            error = target_speed - current_speed
            throttle = speed_controller.step(error, 0.1)
            
            control = carla.VehicleControl()
            if throttle > 0:
                control.throttle = min(0.7, throttle) # Ani hızlanmayı kestik
                control.brake = 0.0
            else:
                control.brake = min(1.0, abs(throttle))
                control.throttle = 0.0
            
            control.steer = 0.0
            vehicle.apply_control(control)

            print(f"Hız: {current_speed:.2f} km/h | Gaz: {control.throttle:.2f}", end='\r')
            world.wait_for_tick()

    except KeyboardInterrupt:
        print("\nDurduruldu.")
    finally:
        for actor in actor_list:
            actor.destroy()

if __name__ == '__main__':
    import random # Random kütüphanesini burada eklediğimden emin ol
    main()