import time
import board
import busio
from digitalio import DigitalInOut
import adafruit_rfm69

# === CONFIGURATION ===
ROBOT_ID = 2  # Change per robot

# === RFM69 SETUP ===
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm69 = adafruit_rfm69.RFM69(spi, CS, RESET, 915.0)

print(f"RFM69 receiver ready (Robot ID: {ROBOT_ID})...")

while True:
    packet = rfm69.receive()
    if packet:
        try:
            msg = str(packet, "utf-8").strip()
            print(f"[RX] {msg}")

            if msg.startswith("TO:"):
                try:
                    target_str, command = msg.split("|", 1)
                    target_id = int(target_str[3:])

                    if target_id == ROBOT_ID or target_id == 255:
                        print(f"Message for this robot ({ROBOT_ID}) → {command}")

                        if command == "START":
                            print("START command received")

                        elif command == "RENDEZVOUS":
                            print("RENDEZVOUS signal received")

                        elif command.startswith("TRACK:"):
                            coords = command.split("TRACK:")[1]
                            x_str, y_str = coords.split(",")
                            x = float(x_str)
                            y = float(y_str)
                            print(f"TRACK received: x={x}, y={y}")
                    else:
                        print(f"Ignoring message for robot {target_id}")

                except Exception as e:
                    print(f"Failed to parse addressed message: {e}")

        except UnicodeDecodeError:
            print("Received non-UTF-8 data")

    time.sleep(0.1)
