import time
import cv2
import numpy as np
import sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def get_color_mask(img_hsv, target_warna):
    warna = target_warna.lower()
    
    if warna == "merah":
        # Merah ada dua karena berada di dua ujung spektrum HSV
        mask1 = cv2.inRange(img_hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(img_hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        return cv2.bitwise_or(mask1, mask2)
    
    elif warna == "biru":
        return cv2.inRange(img_hsv, np.array([100, 150, 50]), np.array([140, 255, 255]))
    
    elif warna == "hijau":
        return cv2.inRange(img_hsv, np.array([40, 50, 50]), np.array([80, 255, 255]))
    
    elif warna == "magenta":
        return cv2.inRange(img_hsv, np.array([140, 100, 100]), np.array([160, 255, 255]))
    
    elif warna == "kuning":
        return cv2.inRange(img_hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))
    
    else:
        # Default kosong jika warna tidak dikenal
        return np.zeros(img_hsv.shape[:2], dtype=np.uint8)

def drive_robot(sim, handles, v_linear, v_angular):
    """Fungsi dasar penggerak roda"""
    jarak_roda = 0.381
    radius_roda = 0.0975
    
    v_kiri = v_linear - (v_angular * jarak_roda / 2.0)
    v_kanan = v_linear + (v_angular * jarak_roda / 2.0)
    
    sim.setJointTargetVelocity(handles["motor_kiri"], v_kiri / radius_roda)
    sim.setJointTargetVelocity(handles["motor_kanan"], v_kanan / radius_roda)

def pergi_ke_warna(sim, handles, target_warna):
    """
    Fungsi cerdas (Visual Servoing) dengan memori blind-spot.
    Robot akan mencari, mendekat, dan berhenti jika menginjak plane (masuk blind spot).
    """
    print(f"Misi KILO: Menuju petak warna {target_warna.upper()}...", flush=True)
    
    sudah_dekat = False 
    
    while True:
        # 1. STATE: Pembacaan Lingkungan (Sensor Kamera)
        state = {
            "target_terlihat": False,
            "luas_area": 0.0,
            "error_x": 0.0
        }
        
        img, res = sim.getVisionSensorImg(handles["kamera"])
        if img:
            img_array = np.frombuffer(img, dtype=np.uint8)
            img_cv = img_array.reshape((res[1], res[0], 3))
            img_cv = cv2.flip(img_cv, 0)
            img_hsv = cv2.cvtColor(cv2.cvtColor(img_cv, cv2.COLOR_RGB2BGR), cv2.COLOR_BGR2HSV)
            
            mask = get_color_mask(img_hsv, target_warna)
            cv2.imshow(f"Mata Robot - Target: {target_warna}", mask)
            cv2.waitKey(1)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                c = max(contours, key=cv2.contourArea)
                state["luas_area"] = cv2.contourArea(c)
                
                if state["luas_area"] > 100:
                    state["target_terlihat"] = True
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        state["error_x"] = cx - (res[0] / 2)
                        
            if state["luas_area"] > 10000:
                sudah_dekat = True

        if state["target_terlihat"]:
            print(f"Mendeteksi target... Luas area: {state['luas_area']}", flush=True)
        else:
            if sudah_dekat:
                print("Target masuk blind spot (terinjak)...", flush=True)
            else:
                print("Target tidak terlihat, robot berputar mencari...", flush=True)

        # 2. INFERENCE / AI: Konversi State menjadi Action
        action = {
            "v_linear": 0.0,
            "v_angular": 0.0,
            "misi_selesai": False
        }
        
        if state["target_terlihat"]:
            if state["luas_area"] > 20000:
                action["misi_selesai"] = True
            else:
                action["v_linear"] = 0.3
                action["v_angular"] = -state["error_x"] * 0.005 
        else:
            if sudah_dekat:
                action["misi_selesai"] = True
            else:
                action["v_linear"] = 0.0
                action["v_angular"] = 0.5 

        # 3. ACTION: Eksekusi ke Joint Space
        if action["misi_selesai"]:
            print(f"SUKSES! Robot telah sampai di petak {target_warna}.", flush=True)
            drive_robot(sim, handles, 0.0, 0.0) # Stop
            break 
        else:
            drive_robot(sim, handles, action["v_linear"], action["v_angular"])
            
        time.sleep(0.05)

def main():
    print("Menghubungkan ke CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    print("Koneksi berhasil!")
    
    handles = {
        "robot" : sim.getObject('/PioneerP3DX'),
        "motor_kiri": sim.getObject('/PioneerP3DX/leftMotor'),
        "motor_kanan": sim.getObject('/PioneerP3DX/rightMotor'),
        "kamera": sim.getObject('/PioneerP3DX/visionSensor')
    }
    
    try:
        if len(sys.argv) > 1:
            perintah_user = sys.argv[1].lower()
        else:
            perintah_user = "hijau"
            
        pergi_ke_warna(sim, handles, perintah_user)
        
    except KeyboardInterrupt:
        print("\nMisi dihentikan.")
    finally:
        drive_robot(sim, handles, 0.0, 0.0)
        cv2.destroyAllWindows()
        print("Sistem dimatikan dengan aman.")

if __name__ == "__main__":
    main()