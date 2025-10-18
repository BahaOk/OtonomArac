import RPi.GPIO as GPIO
import time

class ServoKontrol:
    """
    Tek bir pinden sinyal alarak çalışan direksiyon servo motoru kontrol sınıfı.
    """
    # Sadece tek bir pin parametresi kullanılıyor.
    def __init__(self, pin_servo=17):
        # pin_servo artık direksiyon kontrol pini
        self.pin_direksiyon = pin_servo 
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Sadece tek pini (BCM 17) çıkış olarak ayarlıyoruz.
        GPIO.setup(self.pin_direksiyon, GPIO.OUT)

        self.servo_frekans = 50       # Servo frekansı 50 Hz
        self.merkez_gorev_dongusu = 5.0 # %7.5 Duty Cycle = Merkez (Düz) konum

        # Orijinal koddaki pwm_sag adını koruyoruz, ancak bu artık tek servoyu kontrol eder.
        self.pwm_sag = GPIO.PWM(self.pin_direksiyon, self.servo_frekans)
        
        # Servoyu merkez pozisyonda başlat.
        self.pwm_sag.start(self.merkez_gorev_dongusu)
        print(f"Direksiyon Servosu (BCM {self.pin_direksiyon}) merkezde başlatıldı.")
        time.sleep(0.5) 


    def saga_don(self):
        """Direksiyonu sağa çevirir ve sonra merkeze alır."""
        sag_gorev_dongusu = 6.5 #Sağ için Duty Cycle (%5)
        print("Sağa dön komutu (Direksiyon sağa çevriliyor)")
        
        self.pwm_sag.ChangeDutyCycle(sag_gorev_dongusu)
        time.sleep(0.3)
        #self.dur() # Merkeze geri dön (direksiyonu düzle)


    def sola_don(self):
        """Direksiyonu sola çevirir ve sonra merkeze alır."""
        sol_gorev_dongusu = 3.8 # Sol için Duty Cycle (%10)
        print("Sola dön komutu (Direksiyon sola çevriliyor)")
        
        self.pwm_sag.ChangeDutyCycle(sol_gorev_dongusu)
        time.sleep(0.3)
        #self.dur() # Merkeze geri dön (direksiyonu düzle)

    def duz_git(self):
        """Direksiyonu merkez (düz) konuma getirir."""
        print("Düz git komutu (Direksiyon merkeze alınıyor)")
        
        self.pwm_sag.ChangeDutyCycle(self.merkez_gorev_dongusu)
        time.sleep(0.3)
        
    def dur(self):
        """Direksiyon pozisyonunu merkezde sabitler."""
        self.pwm_sag.ChangeDutyCycle(self.merkez_gorev_dongusu)
        
    def temizle(self):
        """PWM'i durdurur ve GPIO ayarlarını temizler."""
        if self.pwm_sag:
            self.pwm_sag.stop()
            
        GPIO.cleanup()
        print("GPIO Temizlendi.")
