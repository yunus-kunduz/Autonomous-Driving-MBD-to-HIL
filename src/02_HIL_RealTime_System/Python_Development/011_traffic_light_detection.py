import carla
import numpy as np
import cv2
import random
import time

def process_img(image, vehicle):
    # 1. Görüntüyü Al ve Hazırla
    i = np.array(image.raw_data)
    i2 = i.reshape((600, 800, 4))
    frame = np.ascontiguousarray(i2[:, :, :3], dtype=np.uint8)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- 1. KIRMIZI MASKESİ ---
    lower_red1, upper_red1 = np.array([0, 150, 100]), np.array([10, 255, 255])
    lower_red2, upper_red2 = np.array([160, 150, 100]), np.array([180, 255, 255])
    red_mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

    # --- 2. YEŞİL MASKESİ ---
    lower_green, upper_green = np.array([40, 100, 100]), np.array([90, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    # --- 3. ROI (SADECE SARI KUTUNUN İÇİNE BAK) ---
    # Görüntünün ortasında trafik ışıklarının geleceği yeri tarıyoruz
    roi_red = red_mask[150:350, 300:500] 
    roi_green = green_mask[150:350, 300:500]

    red_pixels = np.sum(roi_red == 255)
    green_pixels = np.sum(roi_green == 255)

    control = carla.VehicleControl()
    
    # Karar Mantığı
    if red_pixels > green_pixels and red_pixels > 80:
        control.throttle = 0.0
        control.brake = 1.0
        status, color = "KIRMIZI - BEKLE", (0, 0, 255)
    elif green_pixels > 40:
        control.throttle = 0.5
        control.brake = 0.0
        status, color = "YESIL - GEC", (0, 255, 0)
    else:
        # Işık görünmüyorsa yavaşça devam et
        control.throttle = 0.3
        control.brake = 0.0
        status, color = "ISIK ARANIYOR...", (255, 255, 255)

    vehicle.apply_control(control)

    # Görselleştirme (Debug için sarı kutuyu çiziyoruz)
    cv2.rectangle(frame, (300, 150), (500, 350), (255, 255, 0), 2)
    cv2.putText(frame, status, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    cv2.imshow("Gelistirilmis Trafik Sistemi V2", frame)
    cv2.waitKey(1)

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

        cam_bp = blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '800')
        cam_bp.set_attribute('image_size_y', '600')
        camera = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=1.5, z=2.4)), attach_to=vehicle)
        actor_list.append(camera)

        # Kamera dinlemeye başlıyor
        camera.listen(lambda data: process_img(data, vehicle))

        print("\n>>> PROJE 11 V2: Trafik Sistemi Başlatıldı.")
        print(">>> Sarı kutu ışıkları kapsamalıdır. Kapsamıyorsa koordinatları güncelleyeceğiz.")

        while True:
            # Spectator kamerası (Takip için)
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
            if actor:
                actor.destroy()
        print("Temizlik tamamlandı.")

if __name__ == '__main__':
    main()