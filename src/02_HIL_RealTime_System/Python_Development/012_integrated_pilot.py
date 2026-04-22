import carla
import numpy as np
import cv2
import random

def process_img(image, vehicle):
    # 1. Görüntü Alımı ve Bellek Düzenleme
    i = np.array(image.raw_data)
    i2 = i.reshape((600, 800, 4))
    frame = np.ascontiguousarray(i2[:, :, :3], dtype=np.uint8)

    # --- ŞERİT TAKİP BÖLÜMÜ (STEERING) ---
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    
    # Şeritler için ROI
    mask = np.zeros_like(canny)
    polygon = np.array([[(0, 600), (800, 600), (400, 350)]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_canny = cv2.bitwise_and(canny, mask)
    
    lines = cv2.HoughLinesP(masked_canny, 1, np.pi/180, 20, minLineLength=20, maxLineGap=300)
    
    steer_val = 0.0
    if lines is not None:
        all_x = [line[0][0] for line in lines] + [line[0][2] for line in lines]
        if len(all_x) > 0:
            lane_center_x = np.mean(all_x)
            error = lane_center_x - 400
            steer_val = error / 300.0

    # --- TRAFİK IŞIĞI BÖLÜMÜ (BRAKING) ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 150, 100]); upper_red = np.array([10, 255, 255])
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    
    # Işıklar için ROI (Sadece üst-orta kısım)
    light_roi = red_mask[100:300, 350:450]
    red_pixel_count = np.sum(light_roi == 255)

    # --- KARAR MEKANİZMASI ---
    control = carla.VehicleControl()
    control.steer = max(-1.0, min(1.0, steer_val))
    
    if red_pixel_count > 60:
        control.throttle = 0.0
        control.brake = 1.0
        msg = "KIRMIZI ISIK - DURUYOR"
    else:
        control.throttle = 0.4
        control.brake = 0.0
        msg = "YOL ACIK - SURUSE DEVAM"

    vehicle.apply_control(control)

    # Görselleştirme
    cv2.putText(frame, msg, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Integrated Autonomous Driver", frame)
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

        camera.listen(lambda data: process_img(data, vehicle))

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