
from ultralytics import YOLO
import cv2
from dc_motor import Vehicle

vehicle = Vehicle()
# Eğitilmiş modeli yükle
model = YOLO("best.pt")

# Raspberry Pi kamerasını başlat
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

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

    if max_conf > 0.7 and selected_label:
        print(f"Tespit: {selected_label}")

        if selected_label == 'dur_tabelasi':
            print(" Dur!")
            vehicle.stop()
        elif selected_label == 'isik_kirmizi':
            print(" Kırmızı ışık – dur!")
            vehicle.stop()
        elif selected_label == 'isik_yesil':
            print(" Yeşil ışık – geç!")
            vehicle.forward_normal()
        elif selected_label == 'otuz_hiz':
            print("Hızı 30 yap")
            vehicle.forward_slow()
        elif selected_label == 'elli_hiz':
            print(" Hızı 50 yap")
            vehicle.forward_fast()+

    cv2.imshow("Kamera", frame)
    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
