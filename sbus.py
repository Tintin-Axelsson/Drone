import numpy as np
import serial

_sbusHeader = 0x0F
_sbusFooter = 0x00

ser = serial.Serial('/dev/ttyS0', 100000, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO)
print(ser.name)
print(ser.is_open)

channel = np.zeros(16, dtype=np.uint16)

package = np.array([
    _sbusHeader,
    (channel[0] & 0x07FF),
    (channel[0] & 0x07FF) >> 8 | (channel[1] & 0x07FF) << 3,
    (channel[1] & 0x07FF) >> 5 | (channel[2] & 0x07FF) << 6,
    (channel[2] & 0x07FF) >> 2,
    (channel[2] & 0x07FF) >> 10 | (channel[3] & 0x07FF) << 1,
    (channel[3] & 0x07FF) >> 7 | (channel[4] & 0x07FF) << 4,
    (channel[4] & 0x07FF) >> 4 | (channel[5] & 0x07FF) << 7,
    (channel[5] & 0x07FF) >> 1,
    (channel[5] & 0x07FF) >> 9 | (channel[6] & 0x07FF) << 2,
    (channel[6] & 0x07FF) >> 6 | (channel[7] & 0x07FF) << 5,
    (channel[7] & 0x07FF) >> 3,
    (channel[8] & 0x07FF),
    (channel[8] & 0x07FF) >> 8 | (channel[9] & 0x07FF) << 3,
    (channel[9] & 0x07FF) >> 5 | (channel[10] & 0x07FF) << 6,
    (channel[10] & 0x07FF) >> 2,
    (channel[10] & 0x07FF) >> 10 | (channel[11] & 0x07FF) << 1,
    (channel[11] & 0x07FF) >> 7 | (channel[12] & 0x07FF) << 4,
    (channel[12] & 0x07FF) >> 4 | (channel[13] & 0x07FF) << 7,
    (channel[13] & 0x07FF) >> 1,
    (channel[13] & 0x07FF) >> 9 | (channel[14] & 0x07FF) << 2,
    (channel[14] & 0x07FF) >> 6 | (channel[15] & 0x07FF) << 5,
    (channel[15] & 0x07FF) >> 3,
	0x00, #Flags
	_sbusFooter
], dtype=np.uint8)

try:
    while True:
        ser.write(package)
finally:
    ser.close()
