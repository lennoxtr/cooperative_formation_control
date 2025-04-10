import time
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
import adafruit_rfm69

# --- Buttons Setup ---
btnA = DigitalInOut(board.D5)  # Start
btnB = DigitalInOut(board.D6)  # Rendezvous
btnC = DigitalInOut(board.D12) # Tracking position
btnA.direction = Direction.INPUT
btnB.direction = Direction.INPUT
btnC.direction = Direction.INPUT
btnA.pull = btnB.pull = btnC.pull = Pull.UP

# --- RFM69 Radio Setup ---
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm69 = adafruit_rfm69.RFM69(spi, CS, RESET, 915.0)

def send_command(msg: str):
    print(f"[TX] {msg}")
    rfm69.send(bytes(msg, 'utf-8'))
    time.sleep(0.5)

while True:
    if not btnA.value:
        send_command("START")
    elif not btnB.value:
        send_command("RENDEZVOUS")
    elif not btnC.value:
        # Example static tracking point (could make dynamic later)
        x, y = 1.23, 4.56
        send_command(f"TRACK:{x:.2f},{y:.2f}")

    time.sleep(0.1)
