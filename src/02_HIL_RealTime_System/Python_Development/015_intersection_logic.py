import carla
import random
import time

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    amap = world.get_map() # Haritayı yükle
    
    actor_list = []
    try:
        blueprint_library = world.get_blueprint_library()
        # Proje 15: MAVİ TESLA
        vehicle = world.spawn_actor(blueprint_library.find('vehicle.tesla.model3'), random.choice(amap.get_spawn_points()))
        actor_list.append(vehicle)

        # Otopilotu açıyoruz ama kavşak kararlarını biz manipüle edeceğiz
        vehicle.set_autopilot(True)
        tm = client.get_trafficmanager(8000) # Traffic Manager ile haberleşme

        print("\n>>> PROJE 15: Kavşak Yönetimi Başlatıldı.")
        print(">>> Araç kavşaklara yaklaştığında karar verecek.")

        while True:
            # 1. Aracın o anki Waypoint'ini al
            v_loc = vehicle.get_location()
            waypoint = amap.get_waypoint(v_loc)

            # 2. Eğer araç bir kavşağa girdiyse veya yaklaşmak üzereyse
            if waypoint.is_junction:
                msg = "KAVSAK ALGILANDI: Karar veriliyor..."
                
                # Rastgele bir yön seçelim (Düz, Sağ, Sol)
                # Traffic Manager'a bu araç için rota komutu veriyoruz
                choices = [0, 1, -1] # 0: Düz, 1: Sağ, -1: Sol
                decision = random.choice(choices)
                
                # TM üzerinden araca bu kavşakta ne yapacağını söylüyoruz
                tm.force_lane_change(vehicle, decision > 0) # Sağa veya sola niyetlen
                
                # Hızı kavşak içinde güvenli seviyeye çek (Mühendislik kuralı)
                tm.vehicle_lane_offset(vehicle, 0)
                tm.set_desired_speed(vehicle, 15.0) # 15 km/h hıza düş
                
                color = (255, 255, 0)
            else:
                msg = "NORMAL YOL: Hiz korunuyor."
                tm.set_desired_speed(vehicle, 40.0) # Normal yolda 40 km/h
                color = (0, 255, 0)

            # Debug: Spectator ve Yazı
            world.debug.draw_string(v_loc + carla.Location(z=3), msg, color=carla.Color(color[0], color[1], color[2]), life_time=0.1)
            
            v_trans = vehicle.get_transform()
            world.get_spectator().set_transform(carla.Transform(
                v_trans.location + v_trans.get_forward_vector() * -12 + carla.Location(z=6),
                carla.Rotation(pitch=-20, yaw=v_trans.rotation.yaw)
            ))
            world.wait_for_tick()

    except KeyboardInterrupt:
        print("\nDurduruluyor...")
    finally:
        for actor in actor_list: actor.destroy()

if __name__ == '__main__':
    main()