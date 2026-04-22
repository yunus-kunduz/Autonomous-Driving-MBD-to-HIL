import carla
import numpy as np
import cv2
import random

# Global değişkenler (Dashboard için)
cv_image = np.zeros((400, 400, 3), dtype=np.uint8)
seg_image = np.zeros((400, 400, 3), dtype=np.uint8)
lidar_canvas = np.zeros((400, 400, 3), dtype=np.uint8)

def process_cam(image):
    global cv_image
    i = np.array(image.raw_data)
    i2 = i.reshape((400, 400, 4))
    cv_image = i2[:, :, :3]

def process_seg(image):
    global seg_image
    image.convert(carla.ColorConverter.CityScapesPalette)
    i = np.array(image.raw_data)
    i2 = i.reshape((400, 400, 4))
    seg_image = i2[:, :, :3]

def process_lidar(data):
    global lidar_canvas
    points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    lidar_data = np.array(points[:, :2])
    
    canvas = np.zeros((400, 400, 3), dtype=np.uint8)
    # Izgara (Grid) çizimi
    for r in range(0, 400, 50):
        cv2.circle(canvas, (200, 200), r, (40, 40, 40), 1)

    danger_zone = False
    for p in lidar_data:
        # Mesafe hesaplama: d = sqrt(x^2 + y^2)
        dist = np.sqrt(p[0]**2 + p[1]**2)
        
        x = int(p[0] * 4 + 200)
        y = int(p[1] * 4 + 200)
        
        if 0 <= x < 400 and 0 <= y < 400:
            color = (0, 255, 0)
            if dist < 5.0: # 5 metreden yakınsa kırmızı yap
                color = (0, 0, 255)
                danger_zone = True
            canvas[y, x] = color
            
    cv2.circle(canvas, (200, 200), 5, (255, 255, 255), -1)
    if danger_zone:
        cv2.putText(canvas, "YAKIN ENGEL UYARISI!", (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    lidar_canvas = canvas

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
        vehicle.set_autopilot(True)

        # --- SENSÖRLERİN KURULUMU ---
        # 1. Normal Kamera
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '400')
        cam_bp.set_attribute('image_size_y', '400')
        camera = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=1.5, z=2.4)), attach_to=vehicle)
        actor_list.append(camera)
        camera.listen(lambda data: process_cam(data))

        # 2. Semantic Segmentation (AI)
        seg_bp = blueprint_library.find('sensor.camera.semantic_segmentation')
        seg_bp.set_attribute('image_size_x', '400')
        seg_bp.set_attribute('image_size_y', '400')
        seg_cam = world.spawn_actor(seg_bp, carla.Transform(carla.Location(x=1.5, z=2.4)), attach_to=vehicle)
        actor_list.append(seg_cam)
        seg_cam.listen(lambda data: process_seg(data))

        # 3. LiDAR
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '40')
        lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(x=0, z=2.5)), attach_to=vehicle)
        actor_list.append(lidar)
        lidar.listen(lambda data: process_lidar(data))

        print("\n>>> MASTER DASHBOARD AKTİF.")
        
        while True:
            # Görüntüleri yatayda birleştir (Üst Panel)
            top_row = np.hstack((cv_image, seg_image))
            # LiDAR'ı altına eklemek için boyutları uydur (Alt Panel)
            # Not: Üst sıra 800px genişlikte olduğu için LiDAR'ın yanına boşluk ekliyoruz
            bottom_row = np.hstack((lidar_canvas, np.zeros_like(lidar_canvas)))
            
            # Tüm panelleri dikeyde birleştir
            dashboard = np.vstack((top_row, bottom_row))
            
            cv2.imshow("CARLA Autonomous Engineering Dashboard", dashboard)
            
            # Spectator Takibi
            v_trans = vehicle.get_transform()
            world.get_spectator().set_transform(carla.Transform(
                v_trans.location + v_trans.get_forward_vector() * -10 + carla.Location(z=5),
                carla.Rotation(pitch=-15, yaw=v_trans.rotation.yaw)
            ))
            
            if cv2.waitKey(1) == ord('q'):
                break
            world.wait_for_tick()

    finally:
        cv2.destroyAllWindows()
        for actor in actor_list: actor.destroy()

if __name__ == '__main__':
    main()