# Digi-Pas Inclination Sensor Module

Currently supports only model DWL5500XY.

```
Data received in single mode:
0x61, 0x11, 0x01, 0x12, 0xA8, 0x80, 0x01, 0x13, 0x88, 0xAA, 0x16, 0x46
For DWL5800XY
    Decimal Degree = (((Byte [5]<< 24) + (Byte [4] << 16) + (Byte [3]<< 8) + Byte [2]) - 18000000) / 100000) * 3600
For DWL5500XY
    Decimal Degree = (((Byte [5]<< 24) + (Byte [4] << 16) + (Byte [3]<< 8) + Byte [2]) -18000000) / 100000
For DWL5000XY
    Decimal Degree = (((Byte [5]<< 24) + (Byte [4] << 16) + (Byte [3]<< 8) + Byte [2]) -1800000) / 10000

Data recieved in dual mode:
0x61, 0x22, 0x2D, 0xC6, 0xC0, 0x2D, 0xC6, 0xC0, 0x13, 0x88, 0x31, 0xE2
For DWL5800XY
    Decimal Degree Y = (((Byte [4] << 16) + (Byte [3] << 8) + Byte [2]) - 3000000) / 100000) * 3600
    Decimal Degree X = (((Byte [7] << 16) + (Byte [6] << 8) + Byte [5]) - 3000000) / 100000) * 3600
For DWL5500XY
    Decimal Degree Y = (((Byte [4] << 16) + (Byte [3] << 8) + Byte [2]) - 3000000) / 100000
    Decimal Degree X = (((Byte [7] << 16) + (Byte [6] << 8) + Byte [5]) - 3000000) / 100000
For DWL5000XY
    Decimal Degree Y = (((Byte [4] << 16) + (Byte [3] << 8) + Byte [2]) - 300000) / 10000
    Decimal Degree X = (((Byte [7] << 16) + (Byte [6] << 8) + Byte [5]) - 300000) / 10000
```
