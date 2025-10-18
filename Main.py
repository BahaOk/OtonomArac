import time
import threading
from queue import Queue, Full, Empty

from picamera2 import Picamera2
import cv2

from SeritGoruntu import SeritTakip
from Servo_Kontrol import ServoKontrol
from dc_motor import Vehicle

SHOW_LOCAL = True
velocity = "fast"
karar_son = ""

class CaptureThread(threading.Thread):
    def __init__(self, queue: Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.q = queue
        self.stop_event = stop_event
        self.picam2 = Picamera2()
        cfg = self.picam2.create_video_configuration(main={"size": (640, 480)})
        self.picam2.configure(cfg)

    def run(self):
        self.picam2.start()
        time.sleep(0.5)
        try:
            while not self.stop_event.is_set():
                frame_rgb = self.picam2.capture_array()
                frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                # Kuyruk doluysa en eskisini at, gecikmeyi büyütme
                if self.q.full():
                    try: self.q.get_nowait()
                    except Empty: pass
                try:
                    self.q.put_nowait(frame)
                except Full:
                    pass
        finally:
            try: self.picam2.stop()
            except: pass

def main_loop():
    global velocity, karar_son

    servo = ServoKontrol()
    vehicle = Vehicle()
    serit = SeritTakip()

    frame_q = Queue(maxsize=2)
    stop_event = threading.Event()
    cap_thread = CaptureThread(frame_q, stop_event)
    cap_thread.start()

    frame_count = 0
    start_time = time.time()
    frame_skip = 2  # her 4 karede 1 tanesini işle

    try:
        while not stop_event.is_set():
            # hız modu (sabit)
            v = velocity
            if v == "normal":
                vehicle.forward_normal()
            elif v == "fast":
                vehicle.forward_fast()
            elif v == "slow":
                vehicle.forward_slow()
            else:
                vehicle.stop()

            # Kuyruktan kare al (kısa timeout ile)
            try:
                frame = frame_q.get(timeout=0.2)
            except Empty:
                continue

            frame_count += 1
            if frame_count % frame_skip != 0:
                if SHOW_LOCAL:
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        stop_event.set()
                        break
                continue

            # Şerit tespiti
            lines = serit.get_lines(frame)
            if lines is not None:
                karar = serit.get_steering_decision(lines, frame.shape[1], frame.shape[0])
            else:
                karar = karar_son

            # Servo komutları
            if karar == "Sag":
                print("saga_don")
                servo.saga_don()
            elif karar == "Sol":
                print("sola_don")
                servo.sola_don()
            elif karar == "Duz git":
                print("duz_gittt")
                servo.duz_git()
            else:
                print("Serit bulunamadı, hareket yok (Merkezle)")
                servo.duz_git()

            karar_son = karar

            # FPS/YÖN overlay + görüntüleme
            if SHOW_LOCAL:
                elapsed_time = time.time() - start_time
                fps = frame_count / max(elapsed_time, 1e-6)
                cv2.putText(frame, f"FPS: {fps:.2f}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)
                cv2.putText(frame, f"YON: {karar}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                cv2.imshow("Serit Takibi", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_event.set()
                    break

    except KeyboardInterrupt:
        print("Klavye ile durduruldu.")
        stop_event.set()
    finally:
        stop_event.set()
        cap_thread.join(timeout=2.0)
        vehicle.stop()
        if SHOW_LOCAL:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main_loop()
