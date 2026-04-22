import carla
import numpy as np
import cv2
import random

# --- KONTROL DEĞİŞKENLERİ ---
last_error = 0.0
alpha = 0.3 # Low-pass filtre için
avoidance_state = "FOLLOW" # FOLLOW, SWERVE, RETURN
safety_override = False # Son an freni için

def process_img(image, vehicle, maneuver_steer):
    global last_error, alpha
    i = np.array(image.raw_data)
    i2 = i.reshape((600, 800, 4))
    frame = np.ascontiguousarray(i2[:, :, :3], dtype=np.uint8)

    # 1. Görüntü İşleme (Daha güçlü blur)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(cv2.GaussianBlur(gray, (9, 9), 0), 40, 120)
    
    # Şerit ROI
    mask = np.zeros_like(canny)
    polygon = np.array([[(0, 600), (800, 600), (400, 360)]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_canny = cv2.bitwise_and(canny, mask)
    
    lines = cv2.HoughLinesP(masked_canny, 1, np.pi/180, 25, minLineLength=30, maxLineGap=200)
    
    lane_steer = 0.0
    if lines is not None:
        all_x = [line[0][0] for line in lines] + [line[0][2] for line in lines]
        if len(all_x) > 0:
            error = np.mean(all_x) - 400
            
            # --- PD Kontrolcü (Daha sönümlü) ---
            Kp, Kd = 0.0010, 0.0060
            lane_steer = (Kp * error) + (Kd * (error - last_error))
            last_error = error

    # Manevra açısını şerit takip açısına ekle
    final_steer = lane_steer + maneuver_steer
    return frame, final_steer

def get_lidar_state(data):
    points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    
    # 10m menzil, Z-Filtresi (-1.3)
    relevant_points = points[(points[:, 0] > 1.2) & (points[:, 0] < 10.0) & (points[:, 2] > -1.3)]
    
    # Koridor genişliklerini optimize ettik
    center = relevant_points[(np.abs(relevant_points[:, 1]) < 1.4)]
    left = relevant_points[(relevant_points[:, 1] <= -1.4) & (relevant_points[:, 1] > -3.5)]
    right = relevant_points[(relevant_points[:, 1] >= 1.4) & (relevant_points[:, 1] < 3.5)]
    
    return len(center) > 5, len(left) > 5, len(right) > 5

def main():
    global avoidance_state, safety_override
    client = carla.Client('localhost', 2000); client.set_timeout(10.0)
    world = client.get_world()
    actor_list = []
    
    try:
        blueprint_library = world.get_blueprint_library()
        # GÜVENLİ SPAWN: Boş bir noktada başla
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(blueprint_library.find('vehicle.tesla.model3'), spawn_point)
        actor_list.append(vehicle)

        # Kamera ve LiDAR kurulumları...
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '800'); cam_bp.set_attribute('image_size_y', '600')
        camera = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=1.5, z=2.4)), attach_to=vehicle)
        actor_list.append(camera)

        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '20')
        lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(x=1.0, z=2.0)), attach_to=vehicle)
        actor_list.append(lidar)

        obs_center = obs_left = obs_right = False
        def lidar_callback(data):
            nonlocal obs_center, obs_left, obs_right
            obs_center, obs_left, obs_right = get_lidar_state(data)
        lidar.listen(lambda data: lidar_callback(data))

        def camera_callback(data):
            global avoidance_state, safety_override
            maneuver_steer = 0.0
            
            # --- V2 DİNAMİK MANEVRA MANTIĞI ---
            if avoidance_state == "FOLLOW":
                if obs_center:
                    if not obs_left: # Sola kaç
                        avoidance_state = "SWERVE_LEFT"
                    elif not obs_right: # Sağa kaç
                        avoidance_state = "SWERVE_RIGHT"
                    else: # Kilitlendin, dur
                        avoidance_state = "FOLLOW"
            
            elif avoidance_state == "SWERVE_LEFT":
                maneuver_steer = -0.55
                # Artık önümüz boşsa ve sol tarafımızda da engel kalmadıysa dön
                if not obs_center and not obs_left: avoidance_state = "RETURN"

            elif avoidance_state == "SWERVE_RIGHT":
                maneuver_steer = 0.55
                if not obs_center and not obs_right: avoidance_state = "RETURN"
                
            elif avoidance_state == "RETURN":
                maneuver_steer = -last_error * 0.001 # Hafifçe merkeze yönlen
                if not obs_center and not obs_left and not obs_right: avoidance_state = "FOLLOW"

            # Görüntü işleme ve birleştirme
            frame, final_steer = process_img(data, vehicle, maneuver_steer)
            
            control = carla.VehicleControl()
            # Zikzakları sönümlemek için direksiyonu biraz kısıtla
            control.steer = max(-0.8, min(0.8, final_steer))
            
            # --- FAIL-SAFE FREN SİSTEMİ (Çarpmayı Önler) ---
            if obs_center: # Her ne olursa olsun, önün doluysa frene bas!
                control.throttle, control.brake = 0.0, 1.0
                msg, color = "ENGEL VAR - DUR!", (0, 0, 255)
            else:
                control.throttle = 0.3 if avoidance_state == "FOLLOW" else 0.2
                control.brake = 0.0
                msg, color = f"MOD: {avoidance_state}", (0, 255, 0)
            
            vehicle.apply_control(control)
            cv2.putText(frame, msg, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            cv2.imshow("Manevra V2 - Smooth & Fail-Safe", frame)
            cv2.waitKey(1)

        camera.listen(lambda data: camera_callback(data))
        while True:
            v_trans = vehicle.get_transform()
            world.get_spectator().set_transform(carla.Transform(
                v_trans.location + v_trans.get_forward_vector() * -10 + carla.Location(z=5),
                carla.Rotation(pitch=-15, yaw=v_trans.rotation.yaw)
            ))
            world.wait_for_tick()
    finally:
        cv2.destroyAllWindows()
        for actor in actor_list: actor.destroy()

if __name__ == '__main__':
    main()