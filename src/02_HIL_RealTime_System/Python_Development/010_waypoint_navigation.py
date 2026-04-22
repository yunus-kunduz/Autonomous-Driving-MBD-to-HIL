import carla
import random
import time

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    amap = world.get_map() # Harita bilgisini al
    
    actor_list = []
    try:
        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.find('vehicle.tesla.model3')
        
        # Proje 10: ALTIN TESLA (Final rengi!)
        if bp.has_attribute('color'):
            bp.set_attribute('color', '255,215,0')

        spawn_point = random.choice(amap.get_spawn_points())
        vehicle = world.spawn_actor(bp, spawn_point)
        actor_list.append(vehicle)
        vehicle.set_autopilot(True)

        print("\n>>> PROJE 10: Waypoint Navigation Aktif.")
        print(">>> Yol üzerindeki mavi oklar, aracın beynindeki rotayı temsil eder.")

        while True:
            # 1. Aracın mevcut konumundaki waypoint'i bul
            vehicle_loc = vehicle.get_location()
            current_waypoint = amap.get_waypoint(vehicle_loc)

            # 2. 10 metre ilerideki bir sonraki waypoint'i al
            # 'next' fonksiyonu, yolun devamındaki noktaları liste olarak döndürür
            next_waypoints = current_waypoint.next(10.0)

            if len(next_waypoints) > 0:
                target_waypoint = next_waypoints[0]
                
                # 3. GÖRSELLEŞTİRME (Debug Layer)
                # Yolun üzerine 1 saniye kalacak şekilde bir ok çizelim
                world.debug.draw_arrow(
                    current_waypoint.transform.location + carla.Location(z=0.5),
                    target_waypoint.transform.location + carla.Location(z=0.5),
                    thickness=0.1, arrow_size=0.1,
                    color=carla.Color(0, 255, 255), # Turkuaz oklar
                    life_time=1.0
                )

            # Spectator takibi
            v_trans = vehicle.get_transform()
            world.get_spectator().set_transform(carla.Transform(
                v_trans.location + v_trans.get_forward_vector() * -12 + carla.Location(z=6),
                carla.Rotation(pitch=-20, yaw=v_trans.rotation.yaw)
            ))
            
            world.wait_for_tick()

    except KeyboardInterrupt:
        print("\n>>> Final projesi durduruldu.")
    finally:
        for actor in actor_list: actor.destroy()
        print(">>> Temizlik tamamlandı.")

if __name__ == '__main__':
    main()