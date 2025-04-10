import time
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
import adafruit_rfm69

# --- Buttons Setup ---
btnA = DigitalInOut(board.D5)  # START (broadcast)
btnB = DigitalInOut(board.D6)  # RENDEZVOUS (broadcast)
btnC = DigitalInOut(board.D12) # TRACK (per robot)
btnA.direction = Direction.INPUT
btnB.direction = Direction.INPUT
btnC.direction = Direction.INPUT
btnA.pull = btnB.pull = btnC.pull = Pull.UP

# --- RFM69 Radio Setup ---
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm69 = adafruit_rfm69.RFM69(spi, CS, RESET, 915.0)

def send_command(message: str):
    print(f"[TX] {message}")
    rfm69.send(bytes(message, 'utf-8'))
    time.sleep(0.5)

print("RFM69 sender ready...")

while True:
    if not btnA.value:
        # Broadcast START to all robots
        send_command("TO:255|START")

    elif not btnB.value:
        # Broadcast RENDEZVOUS to all
        send_command("TO:255|RENDEZVOUS")

    elif not btnC.value:
        # Send different TRACK goals to each robot
        targets = {
            1: (1.23, 4.56),
            2: (2.34, 5.67),
        }

        for robot_id, (x, y) in targets.items():
            message = f"TO:{robot_id}|TRACK:{x:.2f},{y:.2f}"
            send_command(message)

    time.sleep(0.1)
