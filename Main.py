#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
from queue import Queue, Full, Empty

from flask import Flask, Response, request, jsonify, render_template_string

from picamera2 import Picamera2
import cv2
import numpy as np

from SeritGoruntu import SeritTakip
from Servo_Kontrol import ServoKontrol
from dc_motor import Vehicle

# ================== AYARLAR ==================
SHOW_LOCAL = False         # HDMI ekranda görüntü göstermek istersen True yap
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 5000

# ================== UZAKTAN KONTROL DURUMU ==================
STATE = {
    "mode": "AUTO",            # "AUTO" | "MANUAL" | "STOP"
    "speed": "fast",           # "slow" | "normal" | "fast" | "stop"
    "last_decision": "Duz git",
    "last_seen": None,
    "have_left": 0,
    "have_right": 0,
    "delta_px": 0,
}

# ================== GLOBAL NESNELER / KUYRUKLAR ==================
app = Flask(__name__)
stop_event = threading.Event()

# Kamera → işleme ham kare
frame_q = Queue(maxsize=1)
# İşleme → yayın overlay kare
annot_q = Queue(maxsize=1)

# Donanım/sınıf nesneleri
servo = None
vehicle = None
serit = None

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
        time.sleep(0.4)
        try:
            while not self.stop_event.is_set():
                frame_rgb = self.picam2.capture_array()
                frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

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

# ================== İŞLEME / SÜRÜŞ THREAD ==================
def processing_thread():
    global servo, vehicle, serit, STATE

    servo = ServoKontrol()
    vehicle = Vehicle()
    serit = SeritTakip(lane_width_px=300, center_deadband_px=40)

    frame_count = 0
    start_time = time.time()

    try:
        while not stop_event.is_set():
            # Hız modu (her döngü)
            spd = STATE["speed"]
            if spd == "normal":
                vehicle.forward_normal()
            elif spd == "fast":
                vehicle.forward_fast()
            elif spd == "slow":
                vehicle.forward_slow()
            else:
                vehicle.stop()

            # Kuyruktan kare al
            try:
                frame = frame_q.get(timeout=0.2)
            except Empty:
                continue

            frame_count += 1
            h, w = frame.shape[:2]

            # Şerit tespiti & karar (AUTO modda)
            karar = STATE["last_decision"]
            lines = serit.get_lines(frame)
            if STATE["mode"] == "AUTO":
                karar = serit.get_steering_decision(lines, w, h)
                if (lines is None or len(lines) < 4) and karar == "Duz git":
                    poly_out = serit.detect_lanes_poly(frame)
                    karar = serit.decision_from_poly(poly_out)

                # Servo komutları (AUTO)
                if karar == "Sag":
                    servo.saga_don()
                elif karar == "Sol":
                    servo.sola_don()
                else:
                    servo.duz_git()

                STATE["last_decision"] = karar

            # Overlay + HUD
            vis = frame.copy()
            # çizgiler
            if lines is not None:
                draw_lanes_on_frame(vis, lines, color=(0, 255, 0), show_segments=False)

            elapsed_time = time.time() - start_time
            fps = frame_count / max(elapsed_time, 1e-6)

            # HUD bilgileri
            STATE["last_seen"]  = serit.last_seen
            STATE["have_left"]  = int(serit.db_have_left)
            STATE["have_right"] = int(serit.db_have_right)
            STATE["delta_px"]   = int(serit.db_delta_px)

            put_hud(vis, f"MODE:{STATE['mode']} SPEED:{STATE['speed']}", 40)
            put_hud(vis, f"YON:{STATE['last_decision']} FPS:{fps:.1f}", 70)
            put_hud(vis, f"L:{STATE['have_left']} R:{STATE['have_right']} Δ:{STATE['delta_px']} LAST:{STATE['last_seen']}", 100)

            # MJPEG kuyruğu
            if annot_q.full():
                try:
                    annot_q.get_nowait()
                except Empty:
                    pass
            try:
                annot_q.put_nowait(vis)
            except Full:
                pass

            # İsteğe bağlı yerel pencere
            if SHOW_LOCAL:
                cv2.imshow("Serit Takibi", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    finally:
        vehicle.stop()
        if SHOW_LOCAL:
            cv2.destroyAllWindows()

# ================== MJPEG ÜRETİCİ ==================
def mjpeg_generator():
    while not stop_event.is_set():
        try:
            frame = annot_q.get(timeout=0.2)
        except Empty:
            continue
        ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')

# ================== FLASK ROUTES ==================
@app.route("/")
def index():
    # Basit gömülü arayüz (ayrı template dosyası istemeden)
    html = """
    <!doctype html>
    <html><head>
      <meta charset="utf-8" />
      <meta name="viewport" content="width=device-width, initial-scale=1" />
      <title>Araba Uzaktan Kontrol</title>
      <style>
        body{background:#111;color:#eee;font-family:sans-serif;margin:0}
        .wrap{max-width:980px;margin:0 auto;padding:16px}
        .grid{display:grid;grid-template-columns:1fr 320px;gap:16px}
        .card{background:#1b1b1b;border-radius:16px;padding:16px;box-shadow:0 6px 20px rgba(0,0,0,.35)}
        img{width:100%;height:auto;background:#000}
        button,select{padding:10px 14px;margin:6px;border-radius:10px;border:0;cursor:pointer}
        .status span{display:inline-block;background:#222;padding:6px 10px;margin:4px;border-radius:10px}
      </style>
    </head><body>
      <div class="wrap">
        <h2>Araba Uzaktan Kontrol</h2>
        <div class="grid">
          <div class="card"><img id="stream" src="/video_feed" alt="video"/></div>
          <div class="card">
            <h3>Kontroller</h3>
            <div>
              <button onclick="setMode('AUTO')">AUTO</button>
              <button onclick="setMode('MANUAL')">MANUAL</button>
              <button onclick="setMode('STOP')">STOP</button>
            </div>
            <div>
              <select id="speedSel" onchange="setSpeed(this.value)">
                <option value="fast">Hız: FAST</option>
                <option value="normal">Hız: NORMAL</option>
                <option value="slow">Hız: SLOW</option>
                <option value="stop">Hız: STOP</option>
              </select>
            </div>
            <div>
              <button onclick="steer('left')">◀ Sol</button>
              <button onclick="steer('center')">• Düz</button>
              <button onclick="steer('right')">Sağ ▶</button>
            </div>
            <div class="status" id="status"></div>
          </div>
        </div>
      </div>
      <script>
        async function postJSON(url, data){
          const r = await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
          const j = await r.json(); show(j); return j;
        }
        function show(s){
          const el = document.getElementById('status');
          el.innerHTML = `
            <span>MODE: ${s.mode}</span>
            <span>SPEED: ${s.speed}</span>
            <span>YON: ${s.last_decision}</span>
            <span>L:${s.have_left} R:${s.have_right} Δ:${s.delta_px}</span>
            <span>LAST:${s.last_seen}</span>
          `;
          const sel = document.getElementById('speedSel');
          sel.value = s.speed || 'fast';
        }
        function setMode(mode){ postJSON('/api/cmd', {mode}); }
        function setSpeed(speed){ postJSON('/api/cmd', {speed}); }
        function steer(steer){ postJSON('/api/cmd', {steer}); }
        // ilk durum
        postJSON('/api/cmd', {});
      </script>
    </body></html>
    """
    return render_template_string(html)

@app.route("/video_feed")
def video_feed():
    return Response(mjpeg_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/api/cmd", methods=["POST"])
def api_cmd():
    """
    JSON Body örnekleri:
      {"mode":"AUTO"} | {"mode":"MANUAL"} | {"mode":"STOP"}
      {"speed":"slow|normal|fast|stop"}
      {"steer":"left|right|center"}  # MANUAL modda
    """
    data = request.get_json(force=True, silent=True) or {}

    # Mod
    if "mode" in data:
        mode = data["mode"].upper()
        if mode in ("AUTO", "MANUAL", "STOP"):
            STATE["mode"] = mode
            if mode == "STOP":
                STATE["speed"] = "stop"
                try: vehicle.stop()
                except Exception: pass

    # Hız
    if "speed" in data:
        spd = data["speed"].lower()
        if spd in ("slow", "normal", "fast", "stop"):
            STATE["speed"] = spd

    # Manuel direksiyon
    if "steer" in data and STATE["mode"] == "MANUAL":
        steer = data["steer"].lower()
        if steer == "left":
            servo.sola_don()
            STATE["last_decision"] = "Sol(M)"
        elif steer == "right":
            servo.saga_don()
            STATE["last_decision"] = "Sag(M)"
        elif steer == "center":
            servo.duz_git()
            STATE["last_decision"] = "Duz(M)"

    return jsonify(STATE)

# ================== THREAD BAŞLATMA ==================
def start_threads():
    t_cam = CaptureThread(frame_q, stop_event)
    t_proc = threading.Thread(target=processing_thread, daemon=True)
    t_cam.start()
    t_proc.start()
    return [t_cam, t_proc]

# ================== ANA ÇALIŞTIRMA ==================
def main():
    threads = start_threads()
    try:
        # Flask sunucusu (threaded=True → aynı anda akış + API)
        app.run(host=HTTP_HOST, port=HTTP_PORT, debug=False, threaded=True)
    finally:
        stop_event.set()
        for t in threads:
            t.join(timeout=2.0)
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == "__main__":
    main()
