from ultralytics import YOLO
import cv2
import time

# servo_motor.py dosyasından ServoKontrol sınıfını ve set_angle fonksiyonunu içe aktarın
from servo_motor import ServoKontrol

# dc_motor.py dosyasından Vehicle sınıfını içe aktarın
from dc_motor import Vehicle

# Eğitilmiş YOLO modelini yükle
model = YOLO("best.pt")

# Raspberry Pi kamerasını başlat
cap = cv2.VideoCapture(0)

# Araç ve Servo nesnelerini oluştur
vehicle = Vehicle()
#servo_kontrol = ServoKontrol()

# Başlangıçta ileri yönde normal hızda git
vehicle.forward_normal()
# Başlangıçta servo'yu varsayılan açısına getir (düz gitmek için)
#servo_kontrol.set_angle(100) # Varsayılan olarak 90 derece düz olarak kabul edelim

last_detection_time = time.time()
detection_cooldown = 2 # Saniye cinsinden bekleme süresi

while True:
    ret, frame = cap.read()
    if not ret:
        print("Kamera okunamadı, çıkılıyor...")
        break

    current_time = time.time()

    # Model ile tahmin yap
    results = model(frame)

    boxes = results[0].boxes
    names = model.names

    max_conf = 0
    selected_label = None

    for box in boxes:
        conf = float(box.conf[0])
        if conf > max_conf:
            max_conf = conf
            cls_id = int(box.cls[0])
            selected_label = names[cls_id]

    if max_conf > 0.7 and selected_label and (current_time - last_detection_time > detection_cooldown):
        print(f"Tespit: {selected_label}")

        if selected_label == 'dur_tabelasi' or selected_label == 'isik_kirmizi':
            print("Dur! Araç durduruluyor.")
            vehicle.stop()
            # Durunca servo'yu varsayılan pozisyona al
            #servo_kontrol.set_angle(100)
            time.sleep(3) # Bir süre dur
            vehicle.forward_normal() # Tekrar harekete geç
            #servo_kontrol.set_angle(100) # Tekrar düz git
        elif selected_label == 'isik_yesil':
            print("Yeşil ışık – geç! Araç ileri gidiyor.")
            vehicle.forward_fast() # Yeşil ışıkta daha hızlı git
            #servo_kontrol.set_angle(100) # Düz git
        elif selected_label == 'otuz_hiz':
            print("Hızı 30 yap. Araç yavaşlatılıyor.")
            vehicle.forward_slow()
            #servo_kontrol.set_angle(100) # Düz git
        elif selected_label == 'elli_hiz':
            print("Hızı 50 yap. Araç hızlandırılıyor.")
            vehicle.forward_normal() # Normal hızı 50 olarak kabul edelim veya daha hızlı bir ayar yapın
            #servo_kontrol.set_angle(100) # Düz git

        # Eğer başka bir mantık eklemek isterseniz, burada sola/sağa dönme için işaretlere bakabilirsiniz.
        # Örneğin, bir "saga_don_isareti" veya "sola_don_isareti" etiketi varsa:
        # elif selected_label == 'saga_don_isareti':
        #     print("Sağa dönülüyor.")
        #     servo_kontrol.set_angle(45) # Sağa dönmek için servo açısını ayarla (deneyerek bulunmalı)
        #     vehicle.sagadon() # DC motorları sağa dönmek için ayarla (kısmi hız farkı)
        #     time.sleep(1) # Dönüş süresi
        #     servo_kontrol.set_angle(90) # Tekrar düz git
        #     vehicle.forward_normal() # Tekrar düz ileri git
        # elif selected_label == 'sola_don_isareti':
        #     print("Sola dönülüyor.")
        #     servo_kontrol.set_angle(135) # Sola dönmek için servo açısını ayarla (deneyerek bulunmalı)
        #     vehicle.soladon() # DC motorları sola dönmek için ayarla
        #     time.sleep(1) # Dönüş süresi
        #     servo_kontrol.set_angle(90) # Tekrar düz git
        #     vehicle.forward_normal() # Tekrar düz ileri git

        last_detection_time = current_time # Son algılama zamanını güncelle

    cv2.imshow("Kamera", frame)
    if cv2.waitKey(1) == ord("q"):
        break

# Temizleme işlemleri
cap.release()
cv2.destroyAllWindows()
vehicle.stop() # Program sonlandığında motorları durdur
#servo_kontrol.set_angle(100) # Program sonlandığında servo'yu varsayılan pozisyona al
