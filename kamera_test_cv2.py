import cv2
import time
from ultralytics import YOLO
from servo_motor import ServoKontrol
from dc_motor import Vehicle

# YOLOv8 modelini yükle
model = YOLO("best.pt")

# Motorları başlat
servo = ServoKontrol()
vehicle = Vehicle()

# OpenCV ile kamerayı başlat
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # genişlik
cap.set(4, 480)  # yükseklik

print("Kamera başlatıldı. Algılama başlıyor...")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Kamera görüntüsü alınamadı.")
            break

        results = model.predict(source=frame, save=False, imgsz=640, conf=0.5)

        labels = results[0].names
        detections = results[0].boxes.cls.cpu().numpy().astype(int)

        if len(detections) > 0:
            for cls in detections:
                label = labels[cls]
                print(f"Algılanan: {label}")

                if label == "dur_tabelasi":
                    vehicle.stop()
                    print("🚦 DUR tabelası: Araç durdu.")
                    time.sleep(2)

                elif label == "isik_kirmizi":
                    vehicle.stop()
                    print("🔴 Kırmızı ışık: Araç durdu.")
                    time.sleep(2)

                elif label == "isik_yesil":
                    vehicle.forward()
                    print("🟢 Yeşil ışık: İleri gidiliyor.")
                    time.sleep(1)

                elif label == "otuz_hiz":
                    vehicle.forward_slow()
                    print("🐢 30 Hız tabelası: Yavaş hareket.")
                    time.sleep(1)

                elif label == "elli_hiz":
                    vehicle.forward_fast()
                    print("🚗 50 Hız tabelası: Hızlı hareket.")
                    time.sleep(1)
        else:
            print("❌ Levha algılanmadı. Normal hızda devam.")
            vehicle.forward()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("❗ Kullanıcı tarafından durduruldu.")

finally:
    cap.release()
    cv2.destroyAllWindows()
    vehicle.stop()
