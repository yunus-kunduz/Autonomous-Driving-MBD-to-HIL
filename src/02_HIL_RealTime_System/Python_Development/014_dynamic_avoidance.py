import carla
import numpy as np
import cv2
import random

# --- GLOBAL DURUM KONTROLÜ ---
last_error = 0.0
avoidance_state = "FOLLOW" # FOLLOW, SWERVE, RETURN
swerve_timer = 0

def process_img(image, vehicle, maneuver_steer):
    global last_error
    i = np.array(image.raw_data)
    i2 = i.reshape((600, 800, 4))
    frame = np.ascontiguousarray(i2[:, :, :3], dtype=np.uint8)

    # Şerit Algılama (Daha stabil olması için blur artırıldı)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(cv2.GaussianBlur(gray, (9, 9), 0), 40, 120)
    mask = np.zeros_like(canny)
    polygon = np.array([[(0, 600), (800, 600), (400, 360)]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_canny = cv2.bitwise_and(canny, mask)
    
    lines = cv2.HoughLinesP(masked_canny, 1, np.pi/180, 30, minLineLength=40, maxLineGap=200)
    
    lane_steer = 0.0
    if lines is not None:
        all_x = [line[0][0] for line in lines] + [line[0][2] for line in lines]
        error = np.mean(all_x) - 400
        # PD Kontrolcü (Zikzakları minimize etmek için katsayılar düşürüldü)
        Kp, Kd = 0.0012, 0.0055
        lane_steer = (Kp * error) + (Kd * (error - last_error))
        last_error = error

    # Eğer manevra yapılıyorsa, şerit takip direksiyonuna manevra açısını ekle
    final_steer = lane_steer + maneuver_steer
    return frame, final_steer

def get_surroundings(data):
    points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    
    # 12 metre menzil, asfaltı görmeyen Z filtresi
    rel_p = points[(points[:, 0] > 1.5) & (points[:, 0] < 12.0) & (points[:, 2] > -1.2)]
    
    center = rel_p[(np.abs(rel_p[:, 1]) < 1.5)]
    left = rel_p[(rel_p[:, 1] <= -1.5) & (rel_p[:, 1] > -4.5)]
    right = rel_p[(rel_p[:, 1] >= 1.5) & (rel_p[:, 1] < 4.5)]
    
    return len(center) > 5, len(left) > 5, len(right) > 5

def main():
    global avoidance_state, swerve_timer
    client = carla.Client('localhost', 2000); client.set_timeout(10.0)
    world = client.get_world()
    actor_list = []
    
    try:
        blueprint_library = world.get_blueprint_library()
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(blueprint_library.find('vehicle.tesla.model3'), spawn_point)
        actor_list.append(vehicle)

        # Kamera ve LiDAR Kurulumu
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
            obs_center, obs_left, obs_right = get_surroundings(data)
        lidar.listen(lambda data: lidar_callback(data))

        def camera_callback(data):
            global avoidance_state, swerve_timer
            maneuver_steer = 0.0
            
            # --- DİNAMİK MANEVRA MANTIĞI ---
            if avoidance_state == "FOLLOW":
                if obs_center:
                    if not obs_left: # Sol boşsa sola kır
                        avoidance_state = "SWERVE"
                        maneuver_steer = -0.6
                        swerve_timer = 20 # 20 tick boyunca manevra yap
                    elif not obs_right: # Sağ boşsa sağa kır
                        avoidance_state = "SWERVE"
                        maneuver_steer = 0.6
                        swerve_timer = 20
                    else: # Her yer kapalıysa dur
                        maneuver_steer = 0.0
                        
            elif avoidance_state == "SWERVE":
                maneuver_steer = -0.5 if not obs_left else 0.5
                swerve_timer -= 1
                if swerve_timer <= 0: avoidance_state = "RETURN"
                
            elif avoidance_state == "RETURN":
                maneuver_steer = 0.2 # Hafifçe merkeze yönlen
                if not obs_center: avoidance_state = "FOLLOW"

            frame, final_steer = process_img(data, vehicle, maneuver_steer)
            
            control = carla.VehicleControl()
            control.steer = max(-1.0, min(1.0, final_steer))
            
            # Engel varsa ve kaçacak yer yoksa dur, varsa yavaşça geç
            if obs_center and avoidance_state == "FOLLOW":
                control.throttle, control.brake = 0.0, 1.0
                msg = "ENGEL - DUR!"
            else:
                control.throttle, control.brake = 0.3, 0.0
                msg = f"MOD: {avoidance_state}"
            
            vehicle.apply_control(control)
            cv2.putText(frame, msg, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Project 14 - Dynamic Obstacle Avoidance", frame)
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