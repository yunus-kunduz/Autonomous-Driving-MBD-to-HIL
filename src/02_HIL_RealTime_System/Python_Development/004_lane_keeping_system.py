import carla
import numpy as np
import cv2
import random

def process_img(image):
    # 1. Ham görüntüyü al (800x600)
    i = np.array(image.raw_data)
    i2 = i.reshape((600, 800, 4))
    frame = i2[:, :, :3]

    # 2. Görüntü İşleme (Preprocessing)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)

    # 3. Bölge Belirleme (Region of Interest - ROI)
    mask = np.zeros_like(canny)
    height, width = canny.shape
    # Aracın önündeki yolu kapsayan üçgen bir maske oluşturuyoruz
    polygon = np.array([[
        (0, height), 
        (width, height), 
        (width // 2, height // 2 + 50)
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_image = cv2.bitwise_and(canny, mask)

    # --- YENİ: GÖRÜNTÜLERİ BİRLEŞTİRME ---
    # Canny görüntüsü siyah-beyaz (tek kanal) olduğu için 
    # yan yana koyabilmek için onu 3 kanallı BGR formatına çeviriyoruz
    canny_colored = cv2.cvtColor(masked_image, cv2.COLOR_GRAY2BGR)
    
    # İki görüntüyü yatayda (horizontal) birleştir
    combined_view = np.hstack((frame, canny_colored))

    # Tek bir geniş pencerede göster
    cv2.imshow("Otonom Sistem: Sol (Ham) | Sag (Islenmis ROI)", combined_view)
    cv2.waitKey(1)
    
    return frame

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    actor_list = []

    try:
        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.find('vehicle.tesla.model3')
        
        # Turuncu Tesla
        if bp.has_attribute('color'):
            bp.set_attribute('color', '255,165,0')

        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp, spawn_point)
        actor_list.append(vehicle)

        # Kamera Ekleme (800x600)
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '800')
        cam_bp.set_attribute('image_size_y', '600')
        cam_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
        actor_list.append(camera)

        # Görüntü işleme fonksiyonunu bağla
        camera.listen(lambda data: process_img(data))

        print("\n>>> Lane Keeping System Başlatıldı.")
        print(">>> Sol tarafta gerçek yolu, sağ tarafta bilgisayarın algıladığı çizgileri göreceksiniz.")

        while True:
            # Sabit hızla düz git (Henüz direksiyon kontrolü eklemedik)
            vehicle.apply_control(carla.VehicleControl(throttle=0.4, steer=0.0))
            
            # Spectator kamerasıyla aracı arkadan takip et
            v_trans = vehicle.get_transform()
            world.get_spectator().set_transform(carla.Transform(
                v_trans.location + v_trans.get_forward_vector() * -10 + carla.Location(z=5),
                carla.Rotation(pitch=-15, yaw=v_trans.rotation.yaw)
            ))
            world.wait_for_tick()

    except KeyboardInterrupt:
        print("\n>>> Program durduruluyor...")
    finally:
        # Pencereleri ve aktörleri temizle
        cv2.destroyAllWindows()
        for actor in actor_list:
            if actor:
                actor.destroy()
        print(">>> Temizlik tamamlandı.")

if __name__ == '__main__':
    main()