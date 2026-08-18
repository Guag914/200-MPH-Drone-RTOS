#IN USART2

import socket
import time

def build_crsf_frame(ch1_roll, ch2_pitch, ch3_yaw, ch4_throttle, ch5_arm):
    temp_frame = bytearray(26)

    temp_frame[0] = 0xC8  #sync header (see code)
    temp_frame[1] = 0x18
    temp_frame[2] = 0x16
    temp_frame[3] = ch1_roll & 0xFF
    temp_frame[4] = ((ch1_roll >> 8) | (ch2_pitch << 3)) & 0xFF
    temp_frame[5] = ((ch2_pitch >> 5) | (ch3_yaw << 6)) & 0xFF
    temp_frame[6] = (ch3_yaw >> 2) & 0xFF
    temp_frame[7] = ((ch3_yaw >> 10) | (ch4_throttle << 1)) & 0xFF
    temp_frame[8] = ((ch4_throttle >> 7) | (ch5_arm << 4)) & 0xFF
    temp_frame[9] = (ch5_arm >> 4) & 0xFF
    temp_frame[25] = 0x00

    return bytes(temp_frame)

def send_frame(socket):
    #hardcoded values for simple testing
    ch1_roll = 992
    ch2_pitch = 992
    ch3_yaw = 992
    ch4_throttle = 350
    ch5_arm = 1500  # > 1000 means armed

    frame = build_crsf_frame(ch1_roll, ch2_pitch, ch3_yaw, ch4_throttle, ch5_arm)
    socket.sendall(frame)

if __name__ == "__main__":
    sock = socket.create_connection(("localhost", 3457))
    while True:
        send_frame(sock)
        time.sleep(0.004)