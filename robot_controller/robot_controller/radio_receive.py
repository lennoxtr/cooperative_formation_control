import time
import board
import busio
from digitalio import DigitalInOut
import adafruit_rfm69

# === CONFIGURATION ===
ROBOT_ID = 1  # Change this per robot: 1, 2, 3...

# === RFM69 SETUP ===
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Create RFM69 object with this robot's unique node ID
rfm69 = adafruit_rfm69.RFM69(spi, CS, RESET, 915.0, node=ROBOT_ID)

# Optional encryption
rfm69.encryption_key = b'\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08'

print(f"RFM69 receiver ready (Node ID: {ROBOT_ID})...")

while True:
    packet = rfm69.receive()
    if packet is not None:
        try:
            message = str(packet, "utf-8").strip()
            sender = rfm69.last_sender
            dest = rfm69.last_destination

            # Only accept messages to me or broadcast (255)
            if dest == ROBOT_ID or dest == 255:
                print(f"\n[RX from {sender}] → {message}")

                if message == "START":
                    print("START command received. System starting...")

                elif message == "RENDEZVOUS":
                    print("RENDEZVOUS signal received.")

                elif message.startswith("TRACK:"):
                    try:
                        coords = message.split("TRACK:")[1]
                        x_str, y_str = coords.split(",")
                        x = float(x_str)
                        y = float(y_str)
                        print(f"🎯 TRACK position received: x={x}, y={y}")
                    except ValueError:
                        print("Invalid TRACK format.")

                else:
                    print(f"Unknown command: {message}")

        except UnicodeDecodeError:
            print("Received non-UTF8 message")

    time.sleep(0.1)
