import cv2
import numpy as np
import math
import time

class SeritTakip:
    """
    - get_lines(frame): HoughLinesP ile doğrusal parçalar (kavis için güçlendirildi).
    - detect_lanes_poly(frame): Sliding-window + 2. derece polinom (kavislerde sağlam).
    - get_steering_decision(...): Her durumda 'Sag' / 'Sol' / 'Duz git' üretir.
      Kısmi şeritlerde sanal şerit tamamlama, tamamen kayıpta recovery (en son görülen tarafa arama).
    """

    def __init__(self,
                 lane_width_px=300,         # sanal şerit için tahmini piksel genişliği
                 center_deadband_px=40,     # merkeze yakınsa 'Duz git' bandı
                 kp_center=0.007):

        self.lane_width_px = lane_width_px
        self.center_deadband_px = center_deadband_px
        self.kp_center = kp_center

        # Recovery durumu
        self.last_seen = None      # "LEFT" | "RIGHT" | "BOTH" | None
        self.lost_since = None

        # HSV eşikleri (beyaz şerit için)
        self.lower_white = np.array([0, 0, 140])   # biraz gevşetildi
        self.upper_white = np.array([180, 80, 255])

        # Debug göstergeleri (HUD için main kullanıyor)
        self.db_have_left = False
        self.db_have_right = False
        self.db_delta_px = 0

    # ---------- Görüntü ön işleme ----------
    def region_of_interest(self, img):
        h, w = img.shape[:2]
        mask = np.zeros_like(img)
        roi_top = int(h * 0.5)  # alt yarıya daha çok odaklan
        polygon = np.array([[
            (0, h),
            (w, h),
            (w, roi_top),
            (0, roi_top)
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_img = cv2.bitwise_and(img, mask)
        return masked_img

    # ---------- Hough tabanlı çizgi çıkarımı (kavis için güçlendirilmiş) ----------
    def get_lines(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        blurred = cv2.GaussianBlur(mask, (5, 5), 0)

        # kavisli yerlerde kesik kenarları birleştirmek için hafif dilate
        kernel = np.ones((3,3), np.uint8)
        edges = cv2.Canny(blurred, 50, 150)
        edges = cv2.dilate(edges, kernel, iterations=1)

        cropped = self.region_of_interest(edges)
        lines = cv2.HoughLinesP(
            cropped, 1, np.pi/180,
            threshold=40,        # 50 -> 40
            minLineLength=25,    # 40 -> 25 (kısa parçaları yakala)
            maxLineGap=150       # 100 -> 150 (parçaları bağla)
        )
        return lines

    # ---------- Yardımcılar (Hough için) ----------
    def _fit_avg_lr(self, lines, frame_h, frame_w):
        """
        Hough segmentlerinden sol/sağ için (slope, intercept) ortalaması çıkar.
        Kavislerde yatay parçalara izin vermek için eğim eşiği gevşetildi,
        alt banda ve alt kesişim x konumuna bakıyoruz.
        """
        if lines is None:
            return False, False, None, None

        left, right = [], []
        y_band_top = int(frame_h * 0.60)   # alt %40
        y_band_bot = frame_h - 1
        xc = frame_w / 2.0

        for l in lines:
            x1, y1, x2, y2 = l[0]
            if max(y1, y2) < y_band_top:   # tamamen üstteyse alma
                continue

            if x2 == x1:
                m = 1e6  # çok dik
            else:
                m = (y2 - y1) / (x2 - x1)
            b = y1 - m * x1

            # Eğim eşiği gevşetildi (0.2). Çok yataysa alt kesişim konumuna göre sınıflandır.
            if abs(m) < 0.2:
                y_eval = y_band_bot
                if abs(m) > 1e-6:
                    x_eval = (y_eval - b) / m
                else:
                    x_eval = (x1 + x2) / 2.0
                if x_eval < xc:
                    left.append((m, b))
                else:
                    right.append((m, b))
            else:
                if m < -0.2:
                    left.append((m, b))
                elif m > 0.2:
                    right.append((m, b))

        have_left  = len(left)  > 0
        have_right = len(right) > 0
        left_avg  = np.average(left,  axis=0) if have_left  else None
        right_avg = np.average(right, axis=0) if have_right else None
        return have_left, have_right, left_avg, right_avg

    @staticmethod
    def _x_at_y(mb, y):
        """ y seviyesinde x = (y - b)/m """
        if mb is None:
            return None
        m, b = mb
        if abs(m) < 1e-6:
            return None
        return (y - b) / m

    # ---------- Polinom (sliding window) tabanlı algılama ----------
    def detect_lanes_poly(self, frame):
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        blurred = cv2.GaussianBlur(mask, (5,5), 0)

        # ikili görüntü
        _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY)

        # alt yarı histogram
        bottom = binary[int(h*0.6):, :]
        histogram = np.sum(bottom, axis=0)

        midpoint = w // 2
        leftx_base  = np.argmax(histogram[:midpoint]) if histogram[:midpoint].any() else None
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint if histogram[midpoint:].any() else None

        if leftx_base is None and rightx_base is None:
            return None  # tamamen yok

        # sliding window parametreleri
        nwindows = 9
        window_height = h // nwindows
        nonzero = binary.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        margin = 50
        minpix = 30

        leftx_current = leftx_base
        rightx_current = rightx_base

        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low  = h - (window + 1) * window_height
            win_y_high = h - window * window_height

            if leftx_current is not None:
                win_xleft_low  = leftx_current - margin
                win_xleft_high = leftx_current + margin
                good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                  (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
                left_lane_inds.append(good_left_inds)
                if len(good_left_inds) > minpix:
                    leftx_current = int(np.mean(nonzerox[good_left_inds]))

            if rightx_current is not None:
                win_xright_low  = rightx_current - margin
                win_xright_high = rightx_current + margin
                good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                   (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
                right_lane_inds.append(good_right_inds)
                if len(good_right_inds) > minpix:
                    rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds  = np.concatenate(left_lane_inds)  if len(left_lane_inds)  else np.array([], dtype=int)
        right_lane_inds = np.concatenate(right_lane_inds) if len(right_lane_inds) else np.array([], dtype=int)

        leftx, lefty   = nonzerox[left_lane_inds],  nonzeroy[left_lane_inds]
        rightx, righty = nonzerox[right_lane_inds], nonzeroy[right_lane_inds]

        left_fit  = np.polyfit(lefty,  leftx,  2) if len(leftx)  > 200 else None
        right_fit = np.polyfit(righty, rightx, 2) if len(rightx) > 200 else None

        return left_fit, right_fit, (h, w)

    def decision_from_poly(self, poly_out):
        if poly_out is None:
            return "Duz git"
        left_fit, right_fit, (h, w) = poly_out
        y_eval = h - 1

        def x_from_fit(fit, y):
            return fit[0]*y*y + fit[1]*y + fit[2] if fit is not None else None

        left_x  = x_from_fit(left_fit, y_eval)
        right_x = x_from_fit(right_fit, y_eval)

        have_left  = left_x  is not None
        have_right = right_x is not None

        if have_left and have_right:
            lane_center = (left_x + right_x)/2.0
        elif have_left:
            lane_center = (left_x + (left_x + self.lane_width_px))/2.0
        elif have_right:
            lane_center = ((right_x - self.lane_width_px) + right_x)/2.0
        else:
            return "Duz git"

        frame_center = w/2.0
        delta = lane_center - frame_center
        self.db_delta_px = int(delta)
        self.db_have_left = have_left
        self.db_have_right = have_right

        if   delta < -self.center_deadband_px: return "Sol"
        elif delta >  self.center_deadband_px: return "Sag"
        else:                                   return "Duz git"

    # ---------- Ana karar ----------
    def get_steering_decision(self, lines, frame_width, frame_height):
        """
        DÖNÜŞ: "Sag" | "Sol" | "Duz git"
        Önce Hough ile dener; zayıf kalırsa main tarafı decision_from_poly ile destekler.
        """
        y_eval = frame_height - 1  # alt kısımda merkez ölçümü daha stabil

        have_left, have_right, left_avg, right_avg = self._fit_avg_lr(lines, frame_height, frame_width)

        # Debug bayraklar
        self.db_have_left = bool(have_left)
        self.db_have_right = bool(have_right)
        self.db_delta_px = 0

        # Durum kaydı (recovery için)
        if have_left and have_right:
            self.last_seen = "BOTH"; self.lost_since = None
        elif have_left:
            self.last_seen = "LEFT"; self.lost_since = None
        elif have_right:
            self.last_seen = "RIGHT"; self.lost_since = None
        else:
            if self.lost_since is None:
                self.lost_since = time.time()

        # 1) İki şerit varsa → gerçek merkez
        if have_left and have_right:
            left_x = self._x_at_y(left_avg, y_eval)
            right_x = self._x_at_y(right_avg, y_eval)
            if left_x is None or right_x is None:
                return "Duz git"
            lane_center = (left_x + right_x) / 2.0
            frame_center = frame_width / 2.0
            delta = lane_center - frame_center
            self.db_delta_px = int(delta)

            if   delta < -self.center_deadband_px: return "Sol"
            elif delta >  self.center_deadband_px: return "Sag"
            else:                                   return "Duz git"

        # 2) Yalnız sol varsa → sağ sanal
        if have_left and not have_right:
            left_x = self._x_at_y(left_avg, y_eval)
            if left_x is None:
                return "Duz git"
            right_x_virtual = left_x + self.lane_width_px
            lane_center = (left_x + right_x_virtual) / 2.0
            frame_center = frame_width / 2.0
            delta = lane_center - frame_center
            self.db_delta_px = int(delta)

            if   delta < -self.center_deadband_px: return "Sol"
            elif delta >  self.center_deadband_px: return "Sag"
            else:                                   return "Sag"   # küçük itme

        # 3) Yalnız sağ varsa → sol sanal
        if have_right and not have_left:
            right_x = self._x_at_y(right_avg, y_eval)
            if right_x is None:
                return "Duz git"
            left_x_virtual = right_x - self.lane_width_px
            lane_center = (left_x_virtual + right_x) / 2.0
            frame_center = frame_width / 2.0
            delta = lane_center - frame_center
            self.db_delta_px = int(delta)

            if   delta < -self.center_deadband_px: return "Sol"
            elif delta >  self.center_deadband_px: return "Sag"
            else:                                   return "Sol"   # küçük itme

        # 4) Hiç şerit yok → recovery
        if self.last_seen == "RIGHT":
            return "Sag"
        elif self.last_seen == "LEFT":
            return "Sol"
        else:
            return "Duz git"
