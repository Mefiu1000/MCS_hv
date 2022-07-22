# MCS_hv

Adresses:

TxID: **0x001A**

RxID: **0x001F**


# Request
1) AIR1(+) and AIR2(-) current value:

|RxID  |2      |0x3D   |0x01   |
|------|-------|-------|-------|
|      |DLC    |READ   |RegID  |

2) AIR1(+) and AIR2(-) status:

|RxID  |2      |0x3D   |0x02   |
|------|-------|-------|-------|
|      |DLC    |READ   |RegID  |

3) Change AIR1(+) and AIR2(-) status:

|RxID                             |4      |       |0x03   |AIR+ status |AIR- status |
|---------------------------------|-------|-------|-------|------------|------------|
|                                 |DLC    |WRITE  |RegID  |data[7-0]   |data[7-0]   |
|AIR1 and AIR2 value:(1-0N, 0-OFF)|       |       |       |            |            |

4) BMS status:

|RxID  |2      |0x3D   |0x04   |
|------|-------|-------|-------|
|      |DLC    |READ   |RegID  |

5) IMD status:

|RxID  |2      |0x3D   |0x05   |
|------|-------|-------|-------|
|      |DLC    |READ   |RegID  |

6) Insulation resistance value:

|RxID  |2      |0x3D   |0x06   |
|------|-------|-------|-------|
|      |DLC    |READ   |RegID  |

7) MAIN status monitor:

|RxID                           |3      |       |0x07   |MAIN status|
|-------------------------------|-------|-------|-------|-----------|
|                               |DLC    |WRITE  |RegID  |data[7-0]  |
|MAIN status value:(1-0N, 0-OFF)|       |       |       |           |

# Respond
1) AIR1(+) and AIR2(-) current value:

|TxID  |3      |0x01   |0xNN        |0xNN        |Range |Units    |
|------|-------|-------|------------|------------|------|---------|
|      |DLC    |RegID  |data[7-0]   |data[7-0]   |8bit  |+/-255   |
|      |       |       |AIR+_current|AIR-_current|      |         |

2) AIR1(+) and AIR2(-) status:

|TxID  |3      |0x02   |0xNN       |0xNN       |Range |Units    |
|------|-------|-------|-----------|-----------|------|---------|
|      |DLC    |RegID  |data[7-0]  |data[7-0]  |8bit  |+/-255   |
|      |       |       |AIR1_status|AIR2_status|      |         |

3) BMS status:

|TxID  |3      |0x04   |0xNN      |Range |Units    |
|------|-------|-------|----------|------|---------|
|      |DLC    |RegID  |data[7-0] |8bit  |+/-255   |

4) IMD status:

|TxID  |3      |0x05   |0xNN      |Range |Units    |
|------|-------|-------|----------|------|---------|
|      |DLC    |RegID  |data[7-0] |8bit  |+/-255   |

5) Insulation resistance value:

|TxID  |3      |0x06   |0xNN      |0xNN       |Range |Units    |
|------|-------|-------|----------|-----------|------|---------|
|      |DLC    |RegID  |data[0-7] |data[8-15] |16bit |0-65536  |

# Error list
1) AIR1(+) and AIR2(-) overcurrent:

|TxID  |2      |0x1D   |0x01   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |

2) AIR1(+) status Error:

|TxID  |2      |0x1D   |0x02   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |

3) AIR2(-) status Error:

|TxID  |2      |0x1D   |0x03   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |

4) IMD short cicruit to supply voltage error:

|TxID  |2      |0x1D   |0x04   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |

5) IMD insulation measurement error:

|TxID  |2      |0x1D   |0x05   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |

6) IMD undervoltage error:

|TxID  |2      |0x1D   |0x06   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |

7) IMD speed start error:

|TxID  |2      |0x1D   |0x07   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |

8) IMD device error:

|TxID  |2      |0x1D   |0x08   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |

9) IMD connection fault error:

|TxID  |2      |0x1D   |0x09   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |

10) IMD malfunction error:

|TxID  |2      |0x1D   |0x0A   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |

11) IMD Shutdown circuit error:

|TxID  |2      |0x1D   |0x0B   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |

12) BMS Shutdown circuit error:

|TxID  |2      |0x1D   |0x0C   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |

13) TSMS connection error:

|TxID  |2      |0x1D   |0x0D   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |

14) MSD/HVD connection error:

|TxID  |2      |0x1D   |0x0E   |
|------|-------|-------|-------|
|      |DLC    |ERROR  |Code   |
