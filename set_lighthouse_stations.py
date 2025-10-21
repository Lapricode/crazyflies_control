import serial
import time
import argparse
import sys


def set_bs_channel(port: str, channel: int, baud: int = 115200, timeout: float = 1.0):
    """
    Set Lighthouse V2 base station channel via USB serial.
    port: e.g. "/dev/ttyACM0" or "COM3"
    channel: 1–16
    """

    print(f"Connecting to {port}...\n")
    try:
        ser = serial.Serial(port, baudrate = baud, timeout = timeout)
    except serial.SerialException as e:
        print(f"❌ Failed to open serial port {port}: {e}")
        sys.exit(1)

    time.sleep(0.1)  # give the port time to settle

    # flush buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # send mode command
    cmd = f"mode {channel}\r\n"
    print(f"→ Sending: \"{cmd.strip()}\"")
    ser.write(cmd.encode("utf-8"))
    time.sleep(0.5)

    # read response
    response = ser.read(200).decode("utf-8", errors = "ignore")
    print(f"← Response after \"mode {channel}\":\n{response}\n")

    if f"mode {channel}" not in response:
        raise Exception(f"Did not respond with \"mode {channel}\" ...")

    # send save command
    print("→ Sending: \"param save\"")
    ser.write(b"param save\r\n")
    time.sleep(0.5)
    response2 = ser.read(200).decode("utf-8", errors = "ignore")
    print(f"← Response after \"param save\":\n{response2}\n")

    ser.close()
    print(f"✅ Done! Base station on {port} set to channel {channel}.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = "Set Lighthouse Base Station V2 channel over USB.")
    parser.add_argument("--port", required = True, help = "Serial port (e.g. /dev/ttyACM0 or COM3)")
    parser.add_argument("--channel", type = int, required = True, help = "Channel number (1–16)")
    parser.add_argument("--baud", type = int, default = 115200, help = "Baud rate (default 115200)")
    args = parser.parse_args()

    if not (1 <= args.channel <= 16):
        print("❌ Channel must be between 1 and 16")
        sys.exit(1)

    try:
        set_bs_channel(args.port, args.channel, baud = args.baud)
        print("Channel set successfully!")
    except Exception as e:
        print(e)
        print("Failed to set channel!")
