import carla
import numpy as np
import cv2
import random
import time

def process_img(image, vehicle):
    # 1. Görüntü Alımı ve Bellek Düzenlemesi (HATA BURADAYDI)
    i = np.array(image.raw_data)
    i2 = i.reshape((600, 800, 4))
    
    # CRITICAL FIX: Görüntüyü bellek için 'contiguous' (sürekli) bir kopyaya çeviriyoruz
    # Böylece OpenCV üzerine çizim yaparken hata vermez.
    frame = np.ascontiguousarray(i2[:, :, :3], dtype=np.uint8)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)

    # 2. ROI (Bölge Belirleme)
    mask = np.zeros_like(canny)
    height, width = canny.shape
    polygon = np.array([[(0, height), (width, height), (width // 2, height // 2 + 50)]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_image = cv2.bitwise_and(canny, mask)

    # 3. HOUGH LINE TRANSFORM (Çizgi Tespiti)
    lines = cv2.HoughLinesP(masked_image, 1, np.pi/180, 20, minLineLength=20, maxLineGap=300)
    
    lane_center_x = width // 2 
    if lines is not None:
        all_x = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            all_x.extend([x1, x2])
            # Artık burada hata vermeyecek:
            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)
        
        if len(all_x) > 0:
            lane_center_x = np.mean(all_x)

    # 4. DİREKSİYON KONTROLÜ
    error = lane_center_x - (width // 2)
    steering = error / 350.0 # Daha yumuşak bir dönüş için 350 yaptım
    steering = max(-1.0, min(1.0, steering))

    # Araca komutu gönder (Bu satır çalıştığı an araba hareket eder)
    vehicle.apply_control(carla.VehicleControl(throttle=0.35, steer=steering))

    # Görselleştirme
    cv2.circle(frame, (int(lane_center_x), height - 50), 10, (0, 255, 0), -1) 
    cv2.imshow("Otonom Surus: Direksiyon Kontrolu Aktif", frame)
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
            bp.set_attribute('color', '255,0,255') 

        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp, spawn_point)
        actor_list.append(vehicle)

        cam_bp = blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '800')
        cam_bp.set_attribute('image_size_y', '600')
        cam_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
        actor_list.append(camera)

        camera.listen(lambda data: process_img(data, vehicle))

        print("\n>>> HATA DÜZELTİLDİ. Otonom sürüş başlıyor...")

        while True:
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
        print("Temizlik tamamlandı.")

if __name__ == '__main__':
    main()