import time
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
import adafruit_rfm69

# --- Buttons Setup ---
btnA = DigitalInOut(board.D5)  # Start (broadcast)
btnB = DigitalInOut(board.D6)  # Rendezvous (broadcast or per node if needed)
btnC = DigitalInOut(board.D12) # Tracking position (per robot)
btnA.direction = Direction.INPUT
btnB.direction = Direction.INPUT
btnC.direction = Direction.INPUT
btnA.pull = btnB.pull = btnC.pull = Pull.UP

# --- RFM69 Radio Setup ---
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm69 = adafruit_rfm69.RFM69(spi, CS, RESET, 915.0)

rfm69.node = 100

def send_to(destination, msg: str):
    rfm69.destination = destination
    print(f"[TX → {destination}] {msg}")
    rfm69.send(bytes(msg, 'utf-8'))
    time.sleep(0.2)

while True:
    if not btnA.value:
        # Broadcast START to all nodes (destination 255 = broadcast)
        send_to(255, "START")

    elif not btnB.value:
        # You can broadcast or target rendezvous if needed
        send_to(255, "RENDEZVOUS")

    elif not btnC.value:
        # Send different TRACK coordinates to robots 1 and 2
        track_data = {
            1: (1.23, 4.56),
            2: (2.34, 5.67),
        }

        for robot_id, (x, y) in track_data.items():
            send_to(robot_id, f"TRACK:{x:.2f},{y:.2f}")

    time.sleep(0.1)
