import carla
import numpy as np
import cv2
import random

# --- KONTROL DEĞİŞKENLERİ ---
last_error = 0.0
filtered_error = 0.0 # Gürültüyü süzmek için

def process_img(image, vehicle):
    global last_error, filtered_error
    i = np.array(image.raw_data)
    i2 = i.reshape((600, 800, 4))
    frame = np.ascontiguousarray(i2[:, :, :3], dtype=np.uint8)

    # 1. Görüntü İşleme
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
    mask = np.zeros_like(canny)
    polygon = np.array([[(0, 600), (800, 600), (400, 360)]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_canny = cv2.bitwise_and(canny, mask)
    
    lines = cv2.HoughLinesP(masked_canny, 1, np.pi/180, 25, minLineLength=30, maxLineGap=250)
    
    steer_val = 0.0
    lane_found = False
    
    if lines is not None:
        all_x = [line[0][0] for line in lines] + [line[0][2] for line in lines]
        if len(all_x) > 0:
            lane_found = True
            raw_error = np.mean(all_x) - 400
            
            # --- LOW PASS FILTER (EEE Tarzı Sinyal İşleme) ---
            # Yeni hatanın sadece %30'unu al, eski hatanın %70'ini koru. 
            # Bu sayede çizgilerdeki anlık sıçramalar direksiyonu titretmez.
            alpha = 0.3 
            filtered_error = (alpha * raw_error) + (1.0 - alpha) * filtered_error
            
            # --- PD KONTROLÖR (Yumuşatılmış Ayarlar) ---
            Kp = 0.0015  # Hassasiyeti biraz düşürdük
            Kd = 0.0080  # Türev etkisini (sönümlemeyi) artırdık
            
            diff_error = filtered_error - last_error
            steer_val = (Kp * filtered_error) + (Kd * diff_error)
            last_error = filtered_error

    return frame, steer_val, lane_found

def get_lidar_state(data):
    points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    relevant_points = points[(points[:, 0] > 1.2) & (points[:, 0] < 10.0) & (points[:, 2] > -1.2)]
    center_obs = relevant_points[(np.abs(relevant_points[:, 1]) < 1.4)]
    left_obs = relevant_points[(relevant_points[:, 1] <= -1.4) & (relevant_points[:, 1] > -2.8)]
    right_obs = relevant_points[(relevant_points[:, 1] >= 1.4) & (relevant_points[:, 1] < 2.8)]
    return len(center_obs) > 5, len(left_obs) > 5, len(right_obs) > 5

def main():
    client = carla.Client('localhost', 2000); client.set_timeout(10.0)
    world = client.get_world()
    actor_list = []
    
    try:
        blueprint_library = world.get_blueprint_library()
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(blueprint_library.find('vehicle.tesla.model3'), spawn_point)
        actor_list.append(vehicle)

        # RGB Kamera & LiDAR kurulumları...
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '800'); cam_bp.set_attribute('image_size_y', '600')
        camera = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=1.5, z=2.4)), attach_to=vehicle)
        actor_list.append(camera)

        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(x=1.0, z=2.0)), attach_to=vehicle)
        actor_list.append(lidar)

        obs_center = obs_left = obs_right = False
        def lidar_callback(data):
            nonlocal obs_center, obs_left, obs_right
            obs_center, obs_left, obs_right = get_lidar_state(data)
        lidar.listen(lambda data: lidar_callback(data))

        def camera_callback(data):
            frame, lane_steer, lane_found = process_img(data, vehicle)
            control = carla.VehicleControl()
            
            # --- DİNAMİK HIZ VE MANEVRA ---
            if obs_center:
                control.throttle, control.brake = 0.0, 1.0
                msg, color = "ENGEL VAR - DUR!", (0,0,255)
            elif obs_left:
                control.throttle, control.steer = 0.2, 0.4 # Sağa kaç
                msg, color = "SOLDA ENGEL!", (255, 255, 0)
            elif obs_right:
                control.throttle, control.steer = 0.2, -0.4 # Sola kaç
                msg, color = "SAGDA ENGEL!", (255, 255, 0)
            else:
                # Direksiyon çok kırıksa yavaşla, düzse hızlan
                control.throttle = 0.35 if abs(lane_steer) < 0.1 else 0.2
                control.steer = lane_steer if lane_found else 0.0
                msg, color = "YOL ACIK - FILTRELI PD", (0, 255, 0)
            
            vehicle.apply_control(control)
            cv2.putText(frame, msg, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            cv2.imshow("V4 - Critically Damped Driver", frame)
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