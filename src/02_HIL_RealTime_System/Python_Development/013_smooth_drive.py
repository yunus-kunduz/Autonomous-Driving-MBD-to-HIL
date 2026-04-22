import carla
import numpy as np
import cv2
import random

last_error = 0.0

def process_img(image, vehicle):
    global last_error
    i = np.array(image.raw_data)
    i2 = i.reshape((600, 800, 4))
    frame = np.ascontiguousarray(i2[:, :, :3], dtype=np.uint8)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
    mask = np.zeros_like(canny)
    polygon = np.array([[(0, 600), (800, 600), (400, 380)]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_canny = cv2.bitwise_and(canny, mask)
    
    lines = cv2.HoughLinesP(masked_canny, 1, np.pi/180, 20, minLineLength=20, maxLineGap=300)
    
    steer_val = 0.0
    lane_found = False
    if lines is not None:
        all_x = [line[0][0] for line in lines] + [line[0][2] for line in lines]
        if len(all_x) > 0:
            lane_found = True
            error = np.mean(all_x) - 400
            Kp, Kd = 0.0025, 0.0065
            diff_error = error - last_error
            steer_val = (Kp * error) + (Kd * diff_error)
            last_error = error

    return frame, steer_val, lane_found

def get_lidar_state(data):
    points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    
    # Genişletilmiş koridor (Direkleri kaçırmamak için 2.5 metreye çıkardık)
    # Z-Filtresi: -1.2 (Yeri görme)
    # X-Menzil: 1.0m - 10.0m
    relevant_points = points[(points[:, 0] > 1.0) & (points[:, 0] < 10.0) & (points[:, 2] > -1.2)]
    
    # Bölgeleri ayır (Y koordinatına göre)
    center_obs = relevant_points[(np.abs(relevant_points[:, 1]) < 1.2)]
    left_obs = relevant_points[(relevant_points[:, 1] <= -1.2) & (relevant_points[:, 1] > -2.5)]
    right_obs = relevant_points[(relevant_points[:, 1] >= 1.2) & (relevant_points[:, 1] < 2.5)]
    
    return len(center_obs) > 5, len(left_obs) > 5, len(right_obs) > 5

def main():
    client = carla.Client('localhost', 2000); client.set_timeout(10.0)
    world = client.get_world()
    
    actor_list = []
    try:
        blueprint_library = world.get_blueprint_library()
        # GÜVENLİ SPAWN: Aracın yolda başlamasını sağlamak için trafiğin olduğu bir nokta seçelim
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(blueprint_library.find('vehicle.tesla.model3'), spawn_point)
        actor_list.append(vehicle)

        cam_bp = blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '800'); cam_bp.set_attribute('image_size_y', '600')
        camera = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=1.5, z=2.4)), attach_to=vehicle)
        actor_list.append(camera)

        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '20')
        lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(x=1.0, z=2.0)), attach_to=vehicle)
        actor_list.append(lidar)

        # Durum değişkenleri
        obs_center = obs_left = obs_right = False

        def lidar_callback(data):
            nonlocal obs_center, obs_left, obs_right
            obs_center, obs_left, obs_right = get_lidar_state(data)
        
        lidar.listen(lambda data: lidar_callback(data))

        def camera_callback(data):
            frame, lane_steer, lane_found = process_img(data, vehicle)
            control = carla.VehicleControl()
            
            # --- HİBRİT KARAR MEKANİZMASI ---
            if obs_center: # Önde tam engel varsa
                control.throttle = 0.0
                control.brake = 1.0
                msg, color = "ACIL FREN!", (0, 0, 255)
            elif obs_left: # Solda direk/engel varsa sağa kır
                control.throttle = 0.2
                control.steer = 0.5
                msg, color = "ENGELDEN KAC (SAGA)", (255, 255, 0)
            elif obs_right: # Sağda direk/engel varsa sola kır
                control.throttle = 0.2
                control.steer = -0.5
                msg, color = "ENGELDEN KAC (SOLA)", (255, 255, 0)
            else: # Yol temizse şerit takibine güven
                control.throttle = 0.4
                control.steer = lane_steer if lane_found else 0.0
                msg, color = "YOL ACIK - PD CONTROL", (0, 255, 0)
            
            vehicle.apply_control(control)
            cv2.putText(frame, msg, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            cv2.imshow("Reactive Autonomous Driving", frame)
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