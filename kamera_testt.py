import time

from picamera2 import Picamera2

import cv2

from ultralytics import YOLO

from servo_motor import ServoKontrol

from dc_motor import Vehicle



# Modeli yükle (önceden eğittiğin best.pt dosyasını 'weights' klasörüne koymuş olman gerekiyor)

model = YOLO("weights/best.pt")



# Donanımı başlat

servo = ServoKontrol()

vehicle = Vehicle()

picam2 = Picamera2()



# Kamera ayarı

video_config = picam2.create_video_configuration(main={"size": (640, 480)})

picam2.configure(video_config)

picam2.start()

time.sleep(2)



print("Sistem hazır. Algılama başlıyor...")



try:

    while True:

        frame = picam2.capture_array()

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



        time.sleep(0.1)



except KeyboardInterrupt:

    print("❗ Durduruldu.")

    vehicle.stop()

    time.sleep(1)
