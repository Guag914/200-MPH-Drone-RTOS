#IN USART3
#GPS does NOT use DMA so a c++ input buffer is not necessary

import socket
import time

def nmea_checksum(sentence_body: str) -> int:
    cs = 0
    for c in sentence_body:
        cs ^= ord(c)
    return cs

def deg_to_ddmm(decimal_deg: float, is_lat: bool) -> str:
    #converts decimal degrees into NMEA's ddmm.mmmm / dddmm.mmmm format
    degrees = int(decimal_deg)
    minutes = (decimal_deg - degrees) * 60
    width = 2 if is_lat else 3  #lat uses 2-digit degrees while lon uses 3-digit
    return f"{degrees:0{width}d}{minutes:07.4f}"

def build_gga_sentence(lat_deg, lat_dir, lon_deg, lon_dir, fix_quality, num_sats, altitude_m):
    lat_str = deg_to_ddmm(lat_deg, is_lat=True)
    lon_str = deg_to_ddmm(lon_deg, is_lat=False)

    body = f"GNGGA,123519,{lat_str},{lat_dir},{lon_str},{lon_dir},{fix_quality},{num_sats:02d},0.9,{altitude_m:.1f},M,46.9,M,,"
    checksum = nmea_checksum(body)
    return f"${body}*{checksum:02X}\r\n".encode()

def build_rmc_sentence(lat_deg, lat_dir, lon_deg, lon_dir, status, speed_knots, heading_deg):
    lat_str = deg_to_ddmm(lat_deg, is_lat=True)
    lon_str = deg_to_ddmm(lon_deg, is_lat=False)

    body = f"GNRMC,123519,{status},{lat_str},{lat_dir},{lon_str},{lon_dir},{speed_knots:.1f},{heading_deg:.1f},230394,003.1,W"
    checksum = nmea_checksum(body)
    return f"${body}*{checksum:02X}\r\n".encode()

def send_frame(sock):
    #hardcoded values for now (use random.randint()/random.uniform() later)
    lat_deg = 40.7128
    lat_dir = 'N'
    lon_deg = 74.0060
    lon_dir = 'W'
    fix_quality = 1 #0: no fix, 1: GPS fix, 2: DGPS fix
    num_sats = 8
    altitude_m = 545.4

    status = 'A' #A: valid, V: void (matches gpsParser statusValid check)
    speed_knots = 22.4
    heading_deg = 84.4

    sock.sendall(build_gga_sentence(lat_deg, lat_dir, lon_deg, lon_dir, fix_quality, num_sats, altitude_m))
    sock.sendall(build_rmc_sentence(lat_deg, lat_dir, lon_deg, lon_dir, status, speed_knots, heading_deg))

if __name__ == "__main__":
    sock = socket.create_connection(("localhost", 3458))
    while True:
        send_frame(sock)
        time.sleep(0.1)