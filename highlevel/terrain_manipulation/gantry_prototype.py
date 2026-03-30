import serial
import time
PORT = "/dev/ttyACM0" # serial path (change the port number based on the computer)
BAUD = 115200 # speed
# waits for Marlin to print "ok" before sending the next set of commands
def read_until_ok(ser, cmd, extra_wait=0.0):
    print(">>", cmd)
    ser.write((cmd + "\n").encode())
    t0 = time.time()
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("<<", line)
        if line.lower().startswith("ok"):
            if extra_wait > 0:
                time.sleep(extra_wait)
            return
        if time.time() - t0 > 5:
            raise TimeoutError(f"Timed out waiting for ok from: {cmd}")

def main():
    ser = serial.Serial(PORT, BAUD, timeout=2)
    time.sleep(2)
    ser.reset_input_buffer()
    # Clear any halted states
    read_until_ok(ser, "M999", extra_wait=0.2)
    # Set low current to 700 mA
    read_until_ok(ser, "M906 X700 Y700 Z700")
    # Movement priming
    read_until_ok(ser, "M17")     # enable steppers
    read_until_ok(ser, "M211 S0") # ignore endstops
    read_until_ok(ser, "G91")     # relative mode
    # for G0 movement commands, the distance is in mm and the speed is in mm/min (for first axis movement)
    read_until_ok(ser, "G0 Z-50 F1200") # movement command using the following format: "G0 X/Y/Z{insert distance} F1{insert speed}"
    # Cleanup (optional)
    read_until_ok(ser, "G90")
    read_until_ok(ser, "M211 S1")
    ser.close()
    print("Done.")

if __name__ == "__main__":
    main()
