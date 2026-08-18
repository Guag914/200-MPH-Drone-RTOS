#IN SPI3

# c++
#int16_t mockacX = (rand() % 600) - 300;
#int16_t mockacY = (rand() % 600) - 300;
#int16_t mockacZ = 2048 + ((rand() % 200) - 100);
#int16_t mockgyX = (rand() % 200) - 100;
#int16_t mockgyY = (rand() % 200) - 100;
#int16_t mockgyZ = (rand() % 200) - 100;

def build_imu_frame(ac_x, ac_y, ac_z, gy_x, gy_y, gy_z):
    temp_frame = bytearray(10)

    temp_frame[0] = ac_x & 0xFF; temp_frame[1] = (ac_x >> 8) & 0xFF
    temp_frame[2] = ac_y & 0xFF; temp_frame[3] = (ac_y >> 8) & 0xFF
    temp_frame[4] = ac_z & 0xFF; temp_frame[5] = (ac_z >> 8) & 0xFF
    temp_frame[6] = gy_x  & 0xFF; temp_frame[7] = (gy_x  >> 8) & 0xFF
    temp_frame[8] = gy_y  & 0xFF; temp_frame[9] = (gy_y  >> 8) & 0xFF
    temp_frame[10] = gy_z  & 0xFF; temp_frame[11] = (gy_z  >> 8) & 0xFF

    return bytes(temp_frame)

def get_frame():
    accel_x = 306 #-300 to 600
    accel_y = 597 #-300 to 600
    accel_z = 2108 #2048 + [-100 to 200]

    gyro_x = -98 #-100 to 200
    gyro_y = -91 #-100 to 200
    gyro_z = 134 #-100 to 200

    return build_crsf_frame(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)

#renode SPI code
if 'tx_buffer' not in globals():
    global tx_buffer, tx_index
    tx_buffer = get_frame()
    tx_index = 0

def Transmit(val):
    global tx_buffer, tx_index

    #fetch next byte from the bytearray
    b = tx_buffer[tx_index]
    tx_index += 1

    #reset frame when all bytes are shifted out
    if tx_index >= len(tx_buffer):
        tx_buffer = get_frame()
        tx_index = 0

    return b

def FinishTransmission():
    global tx_index
    tx_index = 0