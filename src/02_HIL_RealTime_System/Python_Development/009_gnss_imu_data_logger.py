import carla
import random
import time

def process_gnss(data):
    # Enlem ve Boylam verileri
    print(f"LAT: {data.latitude:.6f} | LON: {data.longitude:.6f} | ALT: {data.altitude:.2f}m", end='\r')

def process_imu(data):
    # İvmeölçer ve Jiroskop verileri
    accel = data.accelerometer
    gyro = data.gyroscope
    # Bu veriler özellikle kaza algılama veya stabilite kontrolü için kritiktir
    # print(f"Ivme X: {accel.x:.2f} | Gyro Z: {gyro.z:.2f}")

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    actor_list = []
    try:
        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.find('vehicle.tesla.model3')
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp, spawn_point)
        actor_list.append(vehicle)
        vehicle.set_autopilot(True)

        # 1. GNSS (GPS) Sensörü Kurulumu
        gnss_bp = blueprint_library.find('sensor.other.gnss')
        gnss_transform = carla.Transform(carla.Location(x=1.0, z=2.0))
        gnss = world.spawn_actor(gnss_bp, gnss_transform, attach_to=vehicle)
        actor_list.append(gnss)
        gnss.listen(lambda data: process_gnss(data))

        # 2. IMU Sensörü Kurulumu (Ivme ve Açı)
        imu_bp = blueprint_library.find('sensor.other.imu')
        imu = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)
        actor_list.append(imu)
        imu.listen(lambda data: process_imu(data))

        print("\n>>> TELEMETRI SISTEMI AKTIF. Koordinatlar asagida akıyor:")

        while True:
            # Spectator takibi
            v_trans = vehicle.get_transform()
            world.get_spectator().set_transform(carla.Transform(
                v_trans.location + v_trans.get_forward_vector() * -10 + carla.Location(z=5),
                carla.Rotation(pitch=-15, yaw=v_trans.rotation.yaw)
            ))
            world.wait_for_tick()

    except KeyboardInterrupt:
        print("\nDurduruluyor...")
    finally:
        for actor in actor_list: actor.destroy()

if __name__ == '__main__':
    main()