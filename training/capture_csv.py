import serial
import time
import re

PORT = "COM6"
BAUD = 115200
OUT_CSV = "dataset/board_sensor_data.csv"

CLASSES = [
    (0, "SAFE_STORAGE",      "Leave the board resting flat and still on your desk"),
    (1, "ENV_SPOILAGE_RISK", "Exhale warm, humid breath directly onto the AHT20 sensor"),
    (2, "LID_LEFT_OPEN",     "Shine your phone flashlight directly onto the AP3216C sensor"),
    (3, "TAMPER_OR_FALL",    "Pick up, tilt, and gently shake the board in your hand"),
]

# Matches either CSV mode lines OR Guard mode lines if needed, but we will switch board into CSV mode
CSV_RE = re.compile(r"^\d+,\d+\.\d+,\d+\.\d+,\d+,\d+,-?\d+\.\d+,-?\d+\.\d+,-?\d+\.\d+,-?\d+\.\d+,-?\d+\.\d+,-?\d+\.\d+,\d+$")

def set_label(ser, target_label):
    # Ensure board is in CSV mode and cycle label until it matches target_label
    ser.reset_input_buffer()
    ser.write(b"1\r\n")
    time.sleep(0.3)
    text = ser.read_all().decode(errors="ignore")
    if "GUARD INFERENCE MODE" in text:
        ser.write(b"1\r\n")
        time.sleep(0.3)

    # Check current label from one CSV line
    for _ in range(8):
        line = ser.readline().decode(errors="ignore").strip()
        if CSV_RE.match(line):
            cur = int(line.split(",")[-1])
            while cur != target_label:
                ser.write(b"3\r\n")
                time.sleep(0.25)
                cur = (cur + 1) % 4
            break

def main():
    rows = []
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(1.0)
        for label, name, instruction in CLASSES:
            print(f"\n>>> NEXT: Label {label} ({name})")
            print(f"    Action: {instruction}")
            input("    Start doing the action now, then press ENTER to record 15 seconds...")
            set_label(ser, label)
            ser.reset_input_buffer()

            start = time.time()
            count = 0
            while time.time() - start < 15.0:
                line = ser.readline().decode(errors="ignore").strip()
                if CSV_RE.match(line):
                    parts = line.split(",")
                    parts[-1] = str(label)
                    rows.append(",".join(parts))
                    count += 1
                    print(f"    [{label}] {','.join(parts)}")
            print(f"    Captured {count} rows for {name}.")

        # Return board to Guard Inference Mode
        ser.write(b"1\r\n")

    with open(OUT_CSV, "w", newline="") as f:
        f.write("timestamp_ms,temp_c,humidity_pct,als,ps,ax,ay,az,gx,gy,gz,label\n")
        for r in rows:
            f.write(r + "\n")

    print(f"\n[DONE] Saved {len(rows)} clean rows to {OUT_CSV}!")

if __name__ == "__main__":
    main()