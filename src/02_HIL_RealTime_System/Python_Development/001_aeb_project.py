import carla
import random
import time
import numpy as np

def main():
    # 1. CARLA Sunucusuna Bağlanma
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Aktör listesini tutmak için
    actor_list = []

    try:
        # 2. Araç Oluşturma ve Özelleştirme
        blueprint_library = world.get_blueprint_library()
        # Kesin olarak Tesla Model 3 deniyoruz
        bp = blueprint_library.find('vehicle.tesla.model3')
        
        # ARACI KIRMIZI YAP (Trafikte kolayca bulman için)
        if bp.has_attribute('color'):
            bp.set_attribute('color', '255,0,0')
        
        # Aracı rastgele bir noktada spawn et
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp, spawn_point)
        actor_list.append(vehicle)
        
        # Otopilotu başlat
        vehicle.set_autopilot(True)
        print(f"\n>>> Başarılı: {vehicle.type_id} (KIRMIZI) oluşturuldu.")
        print(">>> Not: Simülatörde kırmızı Tesla'yı arayın.")

        # 3. Engel Algılayıcı Sensör (Obstacle Detector) Ekleme
        obs_bp = blueprint_library.find('sensor.other.obstacle')
        obs_bp.set_attribute('distance', '20') # 20 metre menzil
        obs_bp.set_attribute('hit_radius', '0.5')
        
        # Sensörü ön tampona yerleştir
        sensor_transform = carla.Transform(carla.Location(x=2.5, z=1.0))
        sensor = world.spawn_actor(obs_bp, sensor_transform, attach_to=vehicle)
        actor_list.append(sensor)

        # 4. AEB (Acil Fren) Algoritması
        def obstacle_callback(data):
            target = data.other_actor # Sensörün gördüğü nesne
            distance = data.distance
            
            # 5 Metre kritik sınır (ISO 26262 Mantığı)
            if distance < 5.0:
                print(f"!!! ACİL DURUM: {target.type_id} saptandı! Mesafe: {distance:.2f}m. FREN YAPILIYOR!")
                
                # Aracı durdur
                vehicle.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0, throttle=0.0))
                
                # Güvenlik için otopilotu kapat
                vehicle.set_autopilot(False)
            else:
                # Terminalde kalabalık yapmaması için sadece mesafe bilgisi
                print(f"Yol temiz... Önündeki {target.type_id} ile mesafe: {distance:.2f}m", end='\r')

        # Dinlemeyi başlat
        sensor.listen(lambda data: obstacle_callback(data))

        # 5. Ana Döngü
        print("\nSistem Aktif. Durdurmak için Ctrl+C tuşlarına basın.\n")
        while True:
            world.wait_for_tick()

    except KeyboardInterrupt:
        print("\nKullanıcı durdurdu, çıkılıyor...")
    except Exception as e:
        print(f"Hata: {e}")
    finally:
        # 6. Temizlik
        print("Temizlik yapılıyor...")
        for actor in actor_list:
            if actor is not None:
                actor.destroy()
        print("Bitti.")

if __name__ == '__main__':
    main()