import carla
import random
import time
import numpy as np
import cv2

# Görüntü İşleme Fonksiyonu (Aracın Gözü)
def process_img(image):
    # Ham veriyi görsel matrise (800x600 RGB) çevir
    i = np.array(image.raw_data)
    i2 = i.reshape((600, 800, 4))
    i3 = i2[:, :, :3] # Sadece RGB kanalları
    
    # Canlı kamera penceresini oluştur
    cv2.imshow("Tesla On Cam Kamerasi", i3)
    cv2.waitKey(1)
    return i3/255.0

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
            bp.set_attribute('color', '0,255,0') # Yeşil Tesla

        # Güvenli bir noktada aracı oluştur
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp, spawn_point)
        actor_list.append(vehicle)

        # RGB KAMERA SENSÖRÜ EKLEME
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '800')
        cam_bp.set_attribute('image_size_y', '600')
        cam_bp.set_attribute('fov', '110')

        # Kamerayı dikiz aynasının oraya yerleştir (x=ileri, z=yukarı)
        cam_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
        actor_list.append(camera)

        # Kamerayı dinlemeye başla
        camera.listen(lambda data: process_img(data))

        # Test için otopilotu açalım (Şeritleri izleyelim)
        vehicle.set_autopilot(True)

        print("\n>>> Lane Detection Projesi Baslatildi.")
        print(">>> 'Tesla On Cam Kamerasi' penceresinden yolu izleyebilirsiniz.")

        while True:
            # İzleyici kamerasını (Spectator) araca sabitle
            v_trans = vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                v_trans.location + v_trans.get_forward_vector() * -10 + carla.Location(z=5),
                carla.Rotation(pitch=-15, yaw=v_trans.rotation.yaw)
            ))
            world.wait_for_tick()

    except KeyboardInterrupt:
        print("\nDurduruluyor...")
    finally:
        # Temizlik
        cv2.destroyAllWindows()
        for actor in actor_list:
            actor.destroy()
        print("Temizlik tamamlandi.")

if __name__ == '__main__':
    main()