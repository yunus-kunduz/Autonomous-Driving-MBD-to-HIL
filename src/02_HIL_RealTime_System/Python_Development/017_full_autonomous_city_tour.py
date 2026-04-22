import carla
import random
import time

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    world = client.get_world()
    amap = world.get_map()
    
    actor_list = []
    
    try:
        blueprint_library = world.get_blueprint_library()
        # FINAL ARACI: ALTIN RENKLİ TESLA
        bp = blueprint_library.find('vehicle.tesla.model3')
        if bp.has_attribute('color'):
            bp.set_attribute('color', '255,215,0')

        # Aracı başlangıç noktasına koy
        spawn_points = amap.get_spawn_points()
        start_point = random.choice(spawn_points)
        vehicle = world.spawn_actor(bp, start_point)
        actor_list.append(vehicle)

        # --- TRAFFIC MANAGER (Yüksek Seviye Beyin) ---
        tm = client.get_trafficmanager(8000)
        vehicle.set_autopilot(True, tm.get_port())

        # GÜVENLİK AYARLARI
        tm.distance_to_leading_vehicle(vehicle, 5.0) 
        tm.ignore_lights_percentage(vehicle, 0)      
        tm.ignore_signs_percentage(vehicle, 0)       
        
        # --- REVİZE EDİLEN SATIR (HIZ AYARI) ---
        # Hız limitinden 20% daha yavaş git (Hız limitinin 80%'i)
        tm.vehicle_percentage_speed_difference(vehicle, 20.0)
        
        tm.vehicle_lane_offset(vehicle, 0)

        # --- HEDEF BELİRLEME ---
        destination = random.choice(spawn_points).location
        world.debug.draw_string(destination, 'HEDEF BURASI!', draw_shadow=True,
                                color=carla.Color(r=255, g=0, b=0), life_time=100.0,
                                persistent_lines=True)

        print(f"\n>>> FINAL PROJESİ V2: Şehir Turu Başladı!")
        print(f">>> Hedef: {destination.x:.1f}, {destination.y:.1f}")

        while True:
            v_loc = vehicle.get_location()
            dist_to_dest = v_loc.distance(destination)

            if dist_to_dest < 5.0:
                print("\n[TEBRİKLER] Hedefe başarıyla ulaşıldı! Görev tamamlandı.")
                break

            # Telemetri
            v_vel = vehicle.get_velocity()
            speed = 3.6 * (v_vel.x**2 + v_vel.y**2 + v_vel.z**2)**0.5
            print(f"Hız: {speed:.1f} km/h | Hedefe Kalan: {dist_to_dest:.1f} m", end='\r')

            # Spectator Takibi
            v_trans = vehicle.get_transform()
            world.get_spectator().set_transform(carla.Transform(
                v_trans.location + v_trans.get_forward_vector() * -12 + carla.Location(z=6),
                carla.Rotation(pitch=-20, yaw=v_trans.rotation.yaw)
            ))
            
            world.wait_for_tick()

    except KeyboardInterrupt:
        print("\nTur yarım kaldı.")
    finally:
        print("\nSistem temizleniyor...")
        for actor in actor_list:
            actor.destroy()
        print("Final Projesi Tamamlandı.")

if __name__ == '__main__':
    main()