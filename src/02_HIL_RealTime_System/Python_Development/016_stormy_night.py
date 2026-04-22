import carla
import random
import time

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    actor_list = []
    try:
        blueprint_library = world.get_blueprint_library()
        # Proje 16: SİYAH TESLA (Gece gizlenmesi için)
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(blueprint_library.find('vehicle.tesla.model3'), spawn_point)
        actor_list.append(vehicle)

        # --- 1. HAVA DURUMU VE GECE AYARI ---
        # Parametreler: (bulut, yağmur, rüzgar, sis, ıslaklık, güneş_açısı)
        # sun_altitude_angle: -90 (gece) ile 90 (öğle) arasıdır. -20 zifiri karanlıktır.
        weather = carla.WeatherParameters(
            cloudiness=80.0,
            precipitation=90.0,       # Şiddetli Yağmur
            precipitation_deposits=80.0, # Yerdeki su birikintileri
            wind_intensity=100.0,
            fog_density=20.0,         # Sis
            sun_altitude_angle=-20.0  # GECE MODU
        )
        world.set_weather(weather)

        # --- 2. ARAÇ FARLARINI YAKMA ---
        # Gece sürüşü için farlar hayati önem taşır
        lights = carla.VehicleLightState.All # Tüm ışıkları yak (Farlar, fren, sinyaller)
        vehicle.set_light_state(carla.VehicleLightState(lights))

        vehicle.set_autopilot(True)

        print("\n>>> PROJE 16: Gece ve Fırtına Modu Aktif.")
        print(">>> Yağmurun kamera görüntüsü üzerindeki bozucu etkisini izle!")

        while True:
            v_trans = vehicle.get_transform()
            # Spectator'ı biraz daha yakına alalım ki yağmur efektini görelim
            world.get_spectator().set_transform(carla.Transform(
                v_trans.location + v_trans.get_forward_vector() * -8 + carla.Location(z=4),
                carla.Rotation(pitch=-15, yaw=v_trans.rotation.yaw)
            ))
            world.wait_for_tick()

    except KeyboardInterrupt:
        print("\nDurduruluyor...")
    finally:
        # Hava durumunu normale döndürelim (ClearNoon)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        for actor in actor_list: actor.destroy()

if __name__ == '__main__':
    main()