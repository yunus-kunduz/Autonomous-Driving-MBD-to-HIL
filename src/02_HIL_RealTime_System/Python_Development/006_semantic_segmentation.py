import carla
import numpy as np
import cv2
import random

def process_img(image):
    # DÜZELTME: CityCityPalette yerine CityScapesPalette kullanıyoruz
    image.convert(carla.ColorConverter.CityScapesPalette)
    
    i = np.array(image.raw_data)
    i2 = i.reshape((600, 800, 4))
    frame = i2[:, :, :3]
    
    cv2.imshow("AI Gozu: Semantic Segmentation", frame)
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
            bp.set_attribute('color', '0,0,0')

        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp, spawn_point)
        actor_list.append(vehicle)

        # Semantic Segmentation Sensörü
        seg_bp = blueprint_library.find('sensor.camera.semantic_segmentation')
        seg_bp.set_attribute('image_size_x', '800')
        seg_bp.set_attribute('image_size_y', '600')
        
        transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(seg_bp, transform, attach_to=vehicle)
        actor_list.append(camera)

        camera.listen(lambda data: process_img(data))

        # Autopilotu açıp etrafı izleyelim
        vehicle.set_autopilot(True)

        print("\n>>> HATA GİDERİLDİ: Semantic Segmentation Başlatıldı.")
        print(">>> Gri alanlar yol, mavi alanlar araçlardır.")

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

if __name__ == '__main__':
    main()