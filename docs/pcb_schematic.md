# tangAmp Carrier Board Schematic

Carrier/motherboard connecting Tang Nano 20K + PCM1802 ADC module + PCM5102 DAC module
with pots for user controls and an SPI ADC for reading pot positions.

```
                  +-------------------------+
                  |   Tang Nano 20K         |
                  |   (plugs into J1/J2)    |
                  +-------------------------+
                       |        |        |
         +-------------+        |        +-------------+
         |                      |                      |
         v                      v                      v
  +--------------+      +--------------+      +--------------+
  | PCM1802 ADC  |      | MCP3008 SPI  |      | PCM5102 DAC  |
  |   (J3)       |      |  ADC (U1)    |      |   (J4)       |
  +--------------+      +--------------+      +--------------+
                               |
                        +------+------+
                        |             |
                        v             v
                     Pots (P1-P6)  (guitar I/O handled externally)
```

---

## J1 — Tang Nano 20K Left Header (20 pins, 2.54mm)

| Pin # | Signal     | Connects to                    |
|-------|-----------|--------------------------------|
| 1     | 5V        | Power rail (+5V)               |
| 2     | GND       | Ground rail                    |
| 3     | 3.3V      | 3.3V rail                      |
| 4     | IO_68     | (spare)                        |
| 5     | IO_69     | (spare)                        |
| 6     | IO_71     | (spare)                        |
| 7–20  | —         | (not used, broken out as test points) |

## J2 — Tang Nano 20K Right Header (20 pins, 2.54mm)

| Pin # | Signal (pin name) | Connects to                          |
|-------|------------------|--------------------------------------|
| 1     | 5V               | Power rail (+5V)                     |
| 2     | GND              | Ground rail                          |
| 3     | 3.3V             | 3.3V rail                            |
| 4     | IO_73            | → MCP3008 CS (chip select)           |
| 5     | IO_74            | → MCP3008 DIN (MOSI)                 |
| 6     | IO_75            | ← MCP3008 DOUT (MISO)                |
| 7     | IO_76 (BCK)      | → PCM1802.BCK **AND** → PCM5102.BCK  |
| 8     | IO_77 (LRCK)     | → PCM1802.LRCK **AND** → PCM5102.LRCK |
| 9     | IO_48 (ADC_DOUT) | ← PCM1802.DOUT                       |
| 10    | IO_49 (DAC_DIN)  | → PCM5102.DIN                        |
| 11    | IO_26            | → MCP3008 CLK (SPI SCLK)             |
| 12    | IO_27            | (spare — future SPI CS #2)           |
| 13–20 | —                | (broken out as test points)          |

---

## J3 — PCM1802 ADC Module Header (10 pins, 2.54mm female)

| Pin  | Label   | Connects to                            |
|------|---------|----------------------------------------|
| 1    | VCC     | 5V rail                                |
| 2    | GND     | Ground                                 |
| 3    | DOUT    | Tang Nano pin 48                       |
| 4    | BCK     | Tang Nano pin 76 (shared)              |
| 5    | LRCK    | Tang Nano pin 77 (shared)              |
| 6    | FMT0    | GND (I2S mode)                         |
| 7    | FMT1    | GND                                    |
| 8    | MODE0   | GND (slave mode)                       |
| 9    | MODE1   | GND                                    |
| 10   | SCKI    | GND (no external SCKI)                 |

Input jack: wired off-board by user (VIN-L on the module).

---

## J4 — PCM5102 DAC Module Header (10 pins, 2.54mm female)

| Pin  | Label   | Connects to                            |
|------|---------|----------------------------------------|
| 1    | VIN     | 5V rail                                |
| 2    | GND     | Ground                                 |
| 3    | DIN     | Tang Nano pin 49                       |
| 4    | BCK     | Tang Nano pin 76 (shared)              |
| 5    | LRCK    | Tang Nano pin 77 (shared)              |
| 6    | SCK     | GND (forces internal PLL)              |
| 7    | FMT     | GND (I2S format)                       |
| 8    | XSMT    | 3.3V via 10kΩ pull-up (un-mute)        |
| 9    | DEMP    | GND (de-emphasis off)                  |
| 10   | FLT     | GND (normal filter)                    |

Output jack: wired off-board by user.

---

## U1 — MCP3008 SPI ADC (8-channel, 10-bit)

DIP-16 package. Reads pot positions for tone/volume controls.

| MCP3008 pin | Label    | Connects to                          |
|-------------|----------|--------------------------------------|
| 1           | CH0      | P1 wiper (Volume)                    |
| 2           | CH1      | P2 wiper (Gain)                      |
| 3           | CH2      | P3 wiper (Bass)                      |
| 4           | CH3      | P4 wiper (Mid)                       |
| 5           | CH4      | P5 wiper (Treble)                    |
| 6           | CH5      | P6 wiper (Presence)                  |
| 7           | CH6      | (spare, exposed to header)           |
| 8           | CH7      | (spare, exposed to header)           |
| 9           | DGND     | Ground                               |
| 10          | CS/SHDN  | Tang Nano pin 73                     |
| 11          | DIN      | Tang Nano pin 74                     |
| 12          | DOUT     | Tang Nano pin 75                     |
| 13          | CLK      | Tang Nano pin 26                     |
| 14          | AGND     | Ground                               |
| 15          | VREF     | 3.3V rail (through 100Ω ferrite)     |
| 16          | VDD      | 3.3V rail                            |

**Decoupling:** 100nF ceramic from VDD→GND, 100nF from VREF→AGND, right at the IC pins.

---

## Pots P1–P6 (linear-taper 10kΩ, B10K)

Each pot wired identically:

```
       3.3V ──┬──┐
              │  │  pin 3 (CW end)
              │  ◊
              │  ◊
              ├──┤  pin 2 (wiper) ──── MCP3008 CHx
              │  ◊
              │  ◊
              │  │  pin 1 (CCW end)
        GND ──┴──┘
```

Suggested layout:
- **P1 Volume** → MCP3008 CH0
- **P2 Gain** → CH1
- **P3 Bass** → CH2
- **P4 Mid** → CH3
- **P5 Treble** → CH4
- **P6 Presence** → CH5

Add 100nF cap from each wiper to GND (anti-noise filter, keeps RF out of the ADC).

---

## Power Distribution

```
 USB-C (on Tang Nano) ──► 5V rail (powers PCM1802, PCM5102, board)
                          │
                          └─► 3.3V LDO on Tang Nano → 3.3V rail (MCP3008, pots, XSMT)
```

Decoupling capacitors on power rails:
- 10µF bulk on 5V (near J1/J2)
- 10µF bulk on 3.3V
- 100nF at every IC and module power pin

---

## BOM

| Ref  | Part                                  | Qty | Notes                           |
|------|---------------------------------------|-----|---------------------------------|
| J1,J2| 2.54mm 20-pin female header           | 2   | Tang Nano 20K socket            |
| J3,J4| 2.54mm 10-pin female header           | 2   | PCM module sockets              |
| U1   | MCP3008-I/P (DIP-16)                  | 1   | SPI ADC                         |
| P1–P6| 10kΩ linear pot (PCB mount, B10K)    | 6   | e.g. Alpha RV09 or similar      |
| C1–C2| 10µF electrolytic                     | 2   | Bulk decoupling                 |
| C3–Cn| 100nF ceramic 0603/0805               | ~12 | One per IC power pin + pots     |
| R1   | 10kΩ resistor                         | 1   | XSMT pull-up                    |
| FB1  | Ferrite bead 100Ω @ 100MHz            | 1   | VREF filter                     |

---

## Notes for PCB layout

1. Keep the **I2S clock traces (BCK/LRCK) as short as possible** — ideally route them together with equal length to both modules. Jitter hurts audio quality.
2. **Analog ground and digital ground** can share one plane but keep analog components (pots, MCP3008 analog side) physically separated from the Tang Nano digital activity.
3. Place U1 **close to the pots** to minimize analog trace length.
4. Add **test points / vias** on BCK, LRCK, ADC_DOUT, DAC_DIN so you can scope signals when debugging.
5. The Tang Nano 20K plugs in on **female headers** — don't solder it directly.
6. Label everything on the silkscreen (pot names, test points, connectors).

---

## Expansion headers (optional but cheap to add)

Break out the unused GPIO pins to a 2x8 header so you can add:
- HDMI audio (future)
- More pots
- Footswitches
- MIDI
- OLED display
