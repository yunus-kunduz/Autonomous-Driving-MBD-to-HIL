import carla
import numpy as np
import cv2
import random
import time

# --- GLOBAL KONTROL DEĞİŞKENLERİ ---
last_error = 0.0
filtered_error = 0.0
stuck_timer = 0
avoidance_state = "FOLLOW"  # FOLLOW, SWERVE_LEFT, SWERVE_RIGHT, RECOVERY, LIGHT_STOP

def process_sensors(rgb_image, lidar_data, vehicle):
    global last_error, filtered_error, avoidance_state, stuck_timer
    
    # 1. GÖRÜNTÜ İŞLEME (KAMERA)
    raw_img = np.array(rgb_image.raw_data)
    i2 = raw_img.reshape((600, 800, 4))
    frame = np.ascontiguousarray(i2[:, :, :3], dtype=np.uint8)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # A. Şerit Takibi (Lane Detection)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(cv2.GaussianBlur(gray, (7, 7), 0), 50, 150)
    mask = np.zeros_like(canny)
    polygon = np.array([[(0, 600), (800, 600), (400, 360)]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_canny = cv2.bitwise_and(canny, mask)
    lines = cv2.HoughLinesP(masked_canny, 1, np.pi/180, 25, minLineLength=30, maxLineGap=200)

    lane_steer = 0.0
    if lines is not None:
        all_x = [line[0][0] for line in lines] + [line[0][2] for line in lines]
        if len(all_x) > 0:
            raw_error = np.mean(all_x) - 400
            # Low-pass filter (Sinyal yumuşatma)
            filtered_error = (0.3 * raw_error) + (0.7 * filtered_error)
            # PD Control
            lane_steer = (0.0015 * filtered_error) + (0.0065 * (filtered_error - last_error))
            last_error = filtered_error

    # B. Trafik Işığı Algılama (Traffic Light Detection)
    # Ekranın üst-orta bölgesine odaklan
    light_roi = hsv[100:300, 300:500]
    lower_red = np.array([0, 150, 100]); upper_red = np.array([10, 255, 255])
    red_mask = cv2.inRange(light_roi, lower_red, upper_red)
    red_pixels = np.sum(red_mask == 255)

    # 2. ENGEL ALGILAMA (LiDAR)
    points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    # Z-Filtresi (-1.3): Yeri görme, X-Menzil (10m)
    rel_p = points[(points[:, 0] > 1.2) & (points[:, 0] < 10.0) & (points[:, 2] > -1.3)]
    
    obs_center = len(rel_p[(np.abs(rel_p[:, 1]) < 1.5)]) > 5
    obs_left = len(rel_p[(rel_p[:, 1] <= -1.5) & (rel_p[:, 1] > -3.5)]) > 5
    obs_right = len(rel_p[(rel_p[:, 1] >= 1.5) & (rel_p[:, 1] < 3.5)]) > 5

    # 3. NİHAİ KARAR MEKANİZMASI (DECISION LOGIC)
    control = carla.VehicleControl()
    msg = "YOL ACIK"
    color = (0, 255, 0)
    
    # Öncelik 1: Sıkışma Kontrolü (Stuck Recovery)
    if obs_center: stuck_timer += 1
    else: stuck_timer = max(0, stuck_timer - 1)

    if stuck_timer > 60: avoidance_state = "RECOVERY"

    # Durum Makinesi (State Machine)
    if avoidance_state == "RECOVERY":
        control.throttle = 0.3; control.reverse = True; control.steer = -0.5
        msg, color = "KURTARMA MODU (GERI)", (255, 100, 0)
        if stuck_timer > 120: avoidance_state = "FOLLOW"; stuck_timer = 0
        
    elif red_pixels > 80: # Öncelik 2: Trafik Işığı
        control.throttle = 0.0; control.brake = 1.0
        msg, color = "KIRMIZI ISIK - DUR!", (0, 0, 255)
        
    elif obs_center: # Öncelik 3: Acil Engel
        if not obs_left: avoidance_state = "SWERVE_LEFT"
        elif not obs_right: avoidance_state = "SWERVE_RIGHT"
        else:
            control.throttle = 0.0; control.brake = 1.0
            msg, color = "ENGEL VAR - BEKLE", (0, 0, 255)
            
    elif avoidance_state == "SWERVE_LEFT":
        control.throttle = 0.25; control.steer = -0.6
        msg, color = "ENGELDEN KACIYOR (SOL)", (255, 255, 0)
        if not obs_center and not obs_left: avoidance_state = "FOLLOW"
        
    elif avoidance_state == "SWERVE_RIGHT":
        control.throttle = 0.25; control.steer = 0.6
        msg, color = "ENGELDEN KACIYOR (SAG)", (255, 255, 0)
        if not obs_center and not obs_right: avoidance_state = "FOLLOW"
        
    else: # Normal Sürüş (Lane Following)
        avoidance_state = "FOLLOW"
        control.throttle = 0.4 if abs(lane_steer) < 0.1 else 0.25
        control.steer = lane_steer
        msg, color = "OTONOM SURUS AKTIF", (0, 255, 0)

    vehicle.apply_control(control)

    # Görselleştirme (Debug)
    cv2.rectangle(frame, (300, 100), (500, 300), (255, 255, 0), 2) # Işık Kutusu
    cv2.putText(frame, msg, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    cv2.imshow("PROJE 18: NIHAI OTONOM UNIT", frame)
    cv2.waitKey(1)

def main():
    client = carla.Client('localhost', 2000); client.set_timeout(15.0)
    world = client.get_world()
    amap = world.get_map()
    
    actor_list = []
    try:
        # 1. ARAÇ KURULUMU (ALTIN TESLA)
        bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        if bp.has_attribute('color'): bp.set_attribute('color', '255,215,0')
        vehicle = world.spawn_actor(bp, random.choice(amap.get_spawn_points()))
        actor_list.append(vehicle)

        # 2. SENSÖR KURULUMLARI
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '800'); cam_bp.set_attribute('image_size_y', '600')
        camera = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=1.5, z=2.4)), attach_to=vehicle)
        actor_list.append(camera)

        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '20')
        lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(x=1.0, z=2.0)), attach_to=vehicle)
        actor_list.append(lidar)

        # 3. NAVİGASYON (HEDEF BELİRLEME)
        tm = client.get_trafficmanager(8000)
        vehicle.set_autopilot(True, tm.get_port()) # Global yol planı için autopilot açık
        # Ama kontrolü biz manipüle edeceğimiz için TM ayarlarını optimize ediyoruz
        tm.ignore_lights_percentage(vehicle, 0)
        tm.distance_to_leading_vehicle(vehicle, 4.0)

        # Senkronizasyon ve Callback
        last_lidar_data = None
        def lidar_cb(data): nonlocal last_lidar_data; last_lidar_data = data
        lidar.listen(lambda data: lidar_cb(data))

        def cam_cb(data):
            if last_lidar_data: process_sensors(data, last_lidar_data, vehicle)
        camera.listen(lambda data: cam_cb(data))

        print("\n>>> NIHAI PROJE 18: Tum sistemler entegre edildi.")
        print(">>> Altin Tesla hedefe gidiyor. Her turlu engeli ve isigi kontrol ediyor.")

        while True:
            # Cinematic View
            v_trans = vehicle.get_transform()
            world.get_spectator().set_transform(carla.Transform(
                v_trans.location + v_trans.get_forward_vector() * -12 + carla.Location(z=6),
                carla.Rotation(pitch=-20, yaw=v_trans.rotation.yaw)
            ))
            world.wait_for_tick()

    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        for actor in actor_list: actor.destroy()
        print("\nSistem temizlendi. Final projesi basariyla tamamlandi.")

if __name__ == '__main__':
    main()