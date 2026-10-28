# 2. Wiring

Only the OLED and the 4 buttons are wired. Everything else goes over the USB-C cable.

```
                    Arduino UNO R4 Minima
                 ┌──────────────────────────┐
   USB-C  ◄──────┤ USB          (to the PC: simulator)
                 │                          │
   OLED VCC ─────┤ 5V                       │
   OLED GND ─────┤ GND        D2 ├────── [UP]     ──┐
   OLED SDA ─────┤ A4 (SDA)   D3 ├────── [DOWN]   ──┤
   OLED SCL ─────┤ A5 (SCL)   D4 ├────── [SELECT] ──┤── all four buttons: other leg to GND
                 │            D5 ├────── [BACK]   ──┘
                 │            D1 ├────── (optional) USB-TTL adapter RX, plus GND to GND
                 └──────────────────────────┘
```

| From | To R4 pin | Notes |
|---|---|---|
| OLED **GND** | GND | |
| OLED **VCC** | 5V | Most 0.96" modules accept 3.3–5 V. |
| OLED **SCL** | A5 (or the pin marked SCL near AREF) | Same signal. |
| OLED **SDA** | A4 (or SDA) | Same signal. |
| Button **UP** | D2 | The other leg of each button goes to **GND**. |
| Button **DOWN** | D3 | |
| Button **SELECT** | D4 | |
| Button **BACK** | D5 | |
| (optional) USB-TTL **RX** | D1 (TX) | Telemetry at 115200 baud, plus GND to GND. Never connect the adapter's 5V/3V3 pin. |

## Tips for beginners

- **The OLED pin order differs between sellers** (GND-VCC-SCL-SDA or VCC-GND-SCL-SDA). Read the labels printed on the module, not a picture.
- **4-leg tactile buttons:** the two legs on the *same side* are connected inside. Put the button across the breadboard's middle gap and use two legs that are *diagonal* from each other. Then it can't be shorted all the time.
- The firmware runs fine **without** the OLED or the buttons. If the display isn't found, the yellow **L** LED shows the status instead:

| LED | Meaning |
|---|---|
| slow blink (1 Hz) | waiting for the PC simulator |
| double blink | simulator connected, disarmed |
| on | armed (flying) |
| fast blink | failsafe or crash |

- To change pins, edit `firmware/AeroFC_R4/board_config.h`.
