import carla
import numpy as np
import cv2
import random
import time

# --- KONTROL DEĞİŞKENLERİ ---
last_error = 0.0
avoidance_state = "FOLLOW" 
stuck_timer = 0 # Araç ne kadar süredir duruyor?

def process_img(image, vehicle, maneuver_steer):
    global last_error
    i = np.array(image.raw_data)
    i2 = i.reshape((600, 800, 4))
    frame = np.ascontiguousarray(i2[:, :, :3], dtype=np.uint8)

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
            error = np.mean(all_x) - 400
            Kp, Kd = 0.0015, 0.0060
            lane_steer = (Kp * error) + (Kd * (error - last_error))
            last_error = error

    return frame, lane_steer + maneuver_steer

def get_lidar_state(data):
    points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    rel_p = points[(points[:, 0] > 1.2) & (points[:, 0] < 8.0) & (points[:, 2] > -1.2)]
    
    center = rel_p[(np.abs(rel_p[:, 1]) < 1.4)]
    left = rel_p[(rel_p[:, 1] <= -1.4) & (rel_p[:, 1] > -3.0)]
    right = rel_p[(rel_p[:, 1] >= 1.4) & (rel_p[:, 1] < 3.0)]
    return len(center) > 5, len(left) > 5, len(right) > 5

def main():
    global avoidance_state, stuck_timer
    client = carla.Client('localhost', 2000); client.set_timeout(10.0)
    world = client.get_world()
    actor_list = []
    
    try:
        blueprint_library = world.get_blueprint_library()
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(blueprint_library.find('vehicle.tesla.model3'), spawn_point)
        actor_list.append(vehicle)

        # Kamera ve LiDAR kurulumları
        cam_bp = blueprint_library.find('sensor.camera.rgb')
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
            global avoidance_state, stuck_timer
            maneuver_steer = 0.0
            control = carla.VehicleControl()

            # --- SMART RECOVERY LOGIC ---
            if obs_center:
                stuck_timer += 1
            else:
                stuck_timer = max(0, stuck_timer - 1)

            # Eğer 50 tick boyunca (yaklaşık 2.5 sn) takılı kalmışsak REVERSE moduna geç
            if stuck_timer > 50:
                avoidance_state = "RECOVERY"

            if avoidance_state == "RECOVERY":
                control.throttle = 0.3
                control.reverse = True # GERİ VİTES AKTİF
                control.steer = 0.5 # Geri giderken direksiyonu kır
                msg, color = "RECOVERY: GERI GIDIYOR", (255, 100, 0)
                if stuck_timer > 100: # Yeterince geri gittiyse normale dön
                    avoidance_state = "FOLLOW"
                    stuck_timer = 0
            
            elif avoidance_state == "FOLLOW":
                control.reverse = False
                if obs_center:
                    control.throttle, control.brake = 0.0, 1.0
                    msg, color = "ENGEL: DURUYOR", (0, 0, 255)
                    # Kaçış yönü belirle
                    if not obs_left: avoidance_state = "SWERVE_LEFT"
                    elif not obs_right: avoidance_state = "SWERVE_RIGHT"
                else:
                    control.throttle, control.brake = 0.35, 0.0
                    msg, color = "YOL ACIK", (0, 255, 0)

            elif "SWERVE" in avoidance_state:
                control.reverse = False
                control.throttle = 0.25
                maneuver_steer = -0.6 if "LEFT" in avoidance_state else 0.6
                msg, color = f"MANEVRA: {avoidance_state}", (255, 255, 0)
                if not obs_center: avoidance_state = "FOLLOW"

            frame, final_steer = process_img(data, vehicle, maneuver_steer)
            if avoidance_state != "RECOVERY":
                control.steer = max(-0.8, min(0.8, final_steer))
            
            vehicle.apply_control(control)
            cv2.putText(frame, msg, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            cv2.imshow("Manevra V3 - Smart Recovery", frame)
            cv2.waitKey(1)

        camera.listen(lambda data: camera_callback(data))
        while True:
            world.wait_for_tick()
    finally:
        cv2.destroyAllWindows()
        for actor in actor_list: actor.destroy()

if __name__ == '__main__':
    main()