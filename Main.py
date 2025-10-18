import time
import threading
from queue import Queue, Full, Empty

from picamera2 import Picamera2
import cv2
import numpy as np

from SeritGoruntu import SeritTakip
from Servo_Kontrol import ServoKontrol
from dc_motor import Vehicle

# ================== AYARLAR ==================
SHOW_LOCAL = True          # HDMI ekranda görüntü göstermek için True
velocity = "fast"          # "slow" | "normal" | "fast" | "stop"
karar_son = ""             # son karar (şerit kaybolursa devam etsin)

# ================== GÖRSEL YARDIMCI ==================
def draw_lanes_on_frame(frame, lines, color=(0, 255, 0), show_segments=False):
    if lines is None or len(lines) == 0:
        return

    h, w = frame.shape[:2]
    overlay = frame.copy()

    left_lines, right_lines = [], []

    if show_segments:
        for l in lines:
            x1, y1, x2, y2 = l[0]
            cv2.line(overlay, (x1, y1), (x2, y2), color, 2)

    # Hough sınıflamasıyla aynı mantığı kullan (0.2 eşik, alt band)
    y_band_top = int(h * 0.60)
    y_band_bot = h - 1
    xc = w / 2.0

    def add_line(x1, y1, x2, y2):
        if max(y1, y2) < y_band_top:
            return
        if x2 == x1:
            m = 1e6
        else:
            m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1
        if abs(m) < 0.2:
            y_eval = y_band_bot
            x_eval = (y_eval - b)/m if abs(m) > 1e-6 else (x1+x2)/2.0
            (left_lines if x_eval < xc else right_lines).append((m,b))
        else:
            if m < -0.2: left_lines.append((m,b))
            elif m > 0.2: right_lines.append((m,b))

    for l in lines:
        x1, y1, x2, y2 = l[0]
        add_line(x1, y1, x2, y2)

    if len(left_lines) == 0 or len(right_lines) == 0:
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        return

    left_avg = np.average(left_lines, axis=0)
    right_avg = np.average(right_lines, axis=0)

    def pts_from_mb(m, b, y_top, y_bot):
        if abs(m) < 1e-6:
            return None
        x_top = int((y_top - b) / m)
        x_bot = int((y_bot - b) / m)
        return (x_top, y_top, x_bot, y_bot)

    y_top = int(h * 0.60)
    y_bot = h - 1

    lpts = pts_from_mb(left_avg[0], left_avg[1], y_top, y_bot)
    rpts = pts_from_mb(right_avg[0], right_avg[1], y_top, y_bot)

    if lpts and rpts:
        lx1, ly1, lx2, ly2 = lpts
        rx1, ry1, rx2, ry2 = rpts

        poly = np.array([[lx1, ly1], [lx2, ly2], [rx2, ry2], [rx1, ry1]], dtype=np.int32)
        cv2.fillPoly(overlay, [poly], (0, 255, 0))

        cv2.line(overlay, (lx1, ly1), (lx2, ly2), (0, 200, 0), 6)
        cv2.line(overlay, (rx1, ry1), (rx2, ry2), (0, 200, 0), 6)

        lane_center_x_bot = int((lx2 + rx2) / 2)
        cv2.circle(overlay, (lane_center_x_bot, y_bot), 6, (0, 120, 0), -1)

        cv2.line(overlay, (w // 2, y_bot), (w // 2, y_top), (255, 0, 0), 1)

    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

def put_hud(frame, text, y):
    cv2.putText(frame, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 3)
    cv2.putText(frame, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1)

# ================== KAMERA THREAD ==================
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

                # Kuyruk doluysa en eskisini at (gecikme birikmesin)
                if self.q.full():
                    try:
                        self.q.get_nowait()
                    except Empty:
                        pass
                try:
                    self.q.put_nowait(frame)
                except Full:
                    pass
        finally:
            try:
                self.picam2.stop()
            except Exception:
                pass

# ================== ANA SÜRÜŞ DÖNGÜSÜ ==================
def main_loop():
    global velocity, karar_son

    servo = ServoKontrol()
    vehicle = Vehicle()
    serit = SeritTakip(lane_width_px=300, center_deadband_px=40)

    frame_q = Queue(maxsize=1)       # en güncel kare
    stop_event = threading.Event()
    cap_thread = CaptureThread(frame_q, stop_event)
    cap_thread.start()

    frame_count = 0
    start_time = time.time()
    frame_skip = 1  # her karede karar

    try:
        while not stop_event.is_set():
            # --- Hız modu (sabit) ---
            v = velocity
            if v == "normal":
                vehicle.forward_normal()
            elif v == "fast":
                vehicle.forward_fast()
            elif v == "slow":
                vehicle.forward_slow()
            else:
                vehicle.stop()

            # --- Kuyruktan kare al ---
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

            h, w = frame.shape[:2]

            # --- Şerit tespiti ve karar ---
            lines = serit.get_lines(frame)
            karar = serit.get_steering_decision(lines, w, h)

            # Eğer Hough zayıfsa (ör. kavis), polinom fallback dene
            if (lines is None or len(lines) < 4) and karar == "Duz git":
                poly_out = serit.detect_lanes_poly(frame)
                karar = serit.decision_from_poly(poly_out)

            if karar is None:
                karar = karar_son if karar_son else "Duz git"

            # --- Servo komutları ---
            if karar == "Sag":
                print("saga_don")
                servo.saga_don()
            elif karar == "Sol":
                print("sola_don")
                servo.sola_don()
            elif karar == "Duz git":
                print("duz_git")
                servo.duz_git()
            else:
                print("Bilinmeyen karar:", karar)
          

            karar_son = karar

            # --- FPS/YÖN overlay + ÇİZİM + HUD ---
            if SHOW_LOCAL:
                if lines is not None:
                    draw_lanes_on_frame(frame, lines, color=(0, 255, 0), show_segments=False)

                elapsed_time = time.time() - start_time
                fps = frame_count / max(elapsed_time, 1e-6)

                put_hud(frame, f"FPS: {fps:.2f}", 40)
                put_hud(frame, f"YON: {karar}", 70)
                put_hud(frame, f"L:{int(serit.db_have_left)} R:{int(serit.db_have_right)} Δ:{serit.db_delta_px}", 100)
                put_hud(frame, f"LAST:{serit.last_seen} LOST:{'Y' if serit.lost_since else 'N'}", 130)

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
