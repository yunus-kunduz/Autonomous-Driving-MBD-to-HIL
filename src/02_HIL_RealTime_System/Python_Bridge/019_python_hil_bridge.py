import carla
import serial
import struct
import time
import numpy as np
import cv2

# --- CONFIGURATION ---
SERIAL_PORT = 'COM3' # STM32'nin bağlı olduğu port (Aygıt yöneticisinden bak!)
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    print(f">>> {SERIAL_PORT} üzerinden STM32'ye baglanildi.")
except:
    print("!!! HATA: Seri port acilamadi. Port ismini kontrol et!")
    exit()

def send_to_stm32(lane_error, red_light, obs_bits):
    header = 0xAA
    # STM32'deki struct yapısıyla tam uyumlu paket (B = 1 byte, f = 4 byte float)
    # Format: Header (B), LaneError (f), RedLight (B), ObsFlags (B)
    # Checksum: Basit XOR
    payload = struct.pack('<BfBB', header, lane_error, red_light, obs_bits)
    
    # Checksum hesabı (Payload'un XOR'u)
    checksum = 0
    for b in payload: checksum ^= b
    
    packet = payload + struct.pack('<B', checksum)
    ser.write(packet)

def receive_from_stm32():
    if ser.in_waiting >= 10: # Komut paketi 10 byte (BB + float + float + Checksum)
        data = ser.read(10)
        if data[0] == 0xBB: # Doğru komut başlığı mı?
            # <BffB -> Header(1), Steer(4), Throttle(4), Checksum(1)
            try:
                res = struct.unpack('<BffB', data)
                return res[1], res[2] # Steer ve Throttle değerlerini döndür
            except:
                return None, None
    return None, None

def main():
    client = carla.Client('localhost', 2000); client.set_timeout(10.0)
    world = client.get_world()
    
    # Araç ve Sensör kurulumları (Önceki projelerdeki gibi)
    # ... (Burada Tesla, Kamera ve LiDAR spawn edilecek) ...
    # Kısa tutmak için spawn kısımlarını bildiğini varsayıyorum.
    
    print(">>> HIL Modu Aktif: Kontrol STM32'de!")
    
    try:
        while True:
            # 1. CARLA'dan verileri al (Örn: lane_error=25.5, red=0, obs=1)
            # Bu değerler senin sensör callback'lerinden gelecek
            curr_lane_error = 15.2 
            curr_red_light = 0
            curr_obs_bits = 0x01 # Ön Engel Var
            
            # 2. STM32'ye gönder
            send_to_stm32(curr_lane_error, curr_red_light, curr_obs_bits)
            
            # 3. STM32'den cevap bekle
            steer, throttle = receive_from_stm32()
            
            if steer is not None:
                control = carla.VehicleControl()
                control.steer = steer
                control.throttle = throttle
                # vehicle.apply_control(control) # Tesla'ya uygula
                print(f"STM32 Kararı -> Steer: {steer:.2f}, Throttle: {throttle:.2f}")

            world.wait_for_tick()
            
    finally:
        ser.close()

if __name__ == '__main__':
    main()