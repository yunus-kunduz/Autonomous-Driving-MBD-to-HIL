import carla
import numpy as np
import cv2
import random

def process_lidar(data):
    # LiDAR verisi ham halde (x, y, z, intensity) şeklinde gelir
    points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    
    # Sadece X ve Y koordinatlarını alarak kuş bakışı (Top-down) harita yapalım
    lidar_data = np.array(points[:, :2])
    
    # Görselleştirme için siyah bir tuval oluşturalım
    canvas = np.zeros((600, 600, 3), dtype=np.uint8)
    
    # LiDAR noktalarını resim üzerine çizelim (Ölçekleme yaparak)
    for p in lidar_data:
        x = int(p[0] * 5 + 300) # Merkeze hizala
        y = int(p[1] * 5 + 300)
        if 0 <= x < 600 and 0 <= y < 600:
            canvas[y, x] = (0, 255, 0) # Lazer noktalarını yeşil yap
            
    # Aracın kendisini kırmızı bir nokta olarak merkeze koyalım
    cv2.circle(canvas, (300, 300), 5, (0, 0, 255), -1)
    
    cv2.imshow("LiDAR 2D Point Cloud Map", canvas)
    cv2.waitKey(1)

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    actor_list = []

    try:
        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.find('vehicle.tesla.model3')
        if bp.has_attribute('color'):
            bp.set_attribute('color', '255,255,255') # Beyaz Tesla

        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp, spawn_point)
        actor_list.append(vehicle)

        # LiDAR Sensörü Ayarları
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '50')          # 50 metre menzil
        lidar_bp.set_attribute('channels', '32')       # 32 kanal (lazer sayısı)
        lidar_bp.set_attribute('points_per_second', '50000') # Nokta yoğunluğu
        
        transform = carla.Transform(carla.Location(x=0, z=2.4))
        lidar = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)
        actor_list.append(lidar)

        lidar.listen(lambda data: process_lidar(data))

        # Otopilotta test edelim
        vehicle.set_autopilot(True)

        print("\n>>> PROJE 7: LiDAR Veri Akışı Başladı.")
        print(">>> Siyah penceredeki noktalar aracın etrafındaki 3D dünyadır.")

        while True:
            # Spectator kamerası
            v_trans = vehicle.get_transform()
            world.get_spectator().set_transform(carla.Transform(
                v_trans.location + v_trans.get_forward_vector() * -10 + carla.Location(z=5),
                carla.Rotation(pitch=-15, yaw=v_trans.rotation.yaw)
            ))
            world.wait_for_tick()

    except KeyboardInterrupt:
        print("\nDurduruluyor...")
    finally:
        cv2.destroyAllWindows()
        for actor in actor_list:
            actor.destroy()

if __name__ == '__main__':
    main()