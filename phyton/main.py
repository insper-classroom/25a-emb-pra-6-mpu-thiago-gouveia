import serial
import pyautogui as pg
from time import sleep, time

pg.PAUSE = 0

class Pointer:
    def __init__(self, port, baud_rate=115200, speed=2, delay=50, threshold=0.2):
        """
        port: serial port string
        baud_rate: communication speed
        speed: multiplier for cursor movement
        delay: pause between reads in ms
        threshold: minimum movement (in pixels) to apply
        """
        self.com = serial.Serial(port, baud_rate, timeout=delay/1000)
        self.speed = speed
        self.delay = delay/1000
        self.threshold = threshold
        self.dx_zero = 0
        self.dy_zero = 0

    def parse_data(self, pkt):
        """
        Parse 4-byte packet: [axis, msb, lsb, end]
        Returns: axis (0=x,1=y,2=click), raw signed 16-bit value, end byte
        """
        axis = pkt[0]
        msb = pkt[1]
        lsb = pkt[2]
        end = pkt[3]
        # Combine bytes, big-endian
        raw = (msb << 8) | lsb
        # Convert unsigned to signed
        if raw & 0x8000:
            raw -= 0x10000
        return axis, raw, end

    def calibrate(self, duration=2):
        """
        Collect readings for `duration` seconds to set zero offsets for x and y axes.
        """
        input("Calibração: mantenha o sensor estático e pressione Enter...")
        self.com.reset_input_buffer()
        start = time()
        vals_x = []
        vals_y = []
        while time() - start < duration:
            if self.com.in_waiting < 4:
                sleep(self.delay)
                continue
            pkt = self.com.read(4)
            if len(pkt) != 4:
                continue
            axis, raw, end = self.parse_data(pkt)
            if end != 0xFF:
                continue
            if axis == 0:
                vals_x.append(raw)
            elif axis == 1:
                vals_y.append(raw)
            sleep(self.delay)
        # Compute average zero offsets
        if vals_x:
            self.dx_zero = sum(vals_x) / len(vals_x)
        if vals_y:
            self.dy_zero = sum(vals_y) / len(vals_y)
        print(f"Offsets calibrados: dx_zero={self.dx_zero:.1f}, dy_zero={self.dy_zero:.1f}")

    def start_pointer(self, duration=10):
        """
        Move mouse pointer according to sensor for `duration` seconds.
        """
        self.com.reset_input_buffer()
        start = time()
        while time() - start < duration:
            if self.com.in_waiting < 4:
                sleep(self.delay)
                continue
            pkt = self.com.read(4)
            if len(pkt) != 4:
                continue
            axis, raw, end = self.parse_data(pkt)
            if end != 0xFF:
                continue

            # Subtract zero and scale
            if axis == 0:
                dx = (raw - self.dx_zero) / 10 * self.speed
                if abs(dx) > self.threshold:
                    pg.moveRel(dx, 0, duration=self.delay)
            elif axis == 1:
                dy = (raw - self.dy_zero) / 10 * self.speed
                if abs(dy) > self.threshold:
                    pg.moveRel(0, dy, duration=self.delay)
            elif axis == 2:
                pg.click()

if __name__ == '__main__':
    # Substitua a porta pelo caminho correto que seu mac mostrou
    pt = Pointer('/dev/cu.usbmodem1102', baud_rate=115200, speed=2, delay=50)
    pt.calibrate()
    pt.start_pointer(duration=10)

    pt = Pointer('/dev/cu.usbmodem1102', 115200)
    pt.calibrate()
    pt.start_pointer(10)
