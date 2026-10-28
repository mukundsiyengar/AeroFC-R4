# 1. What you need

## Hardware

| Item | Notes |
|---|---|
| **Arduino UNO R4 Minima** | The flight controller. It talks to the PC over its USB-C port (native USB). |
| **USB-C cable that carries data** | Some cables only charge. If Windows doesn't "ding" when you plug the board in, try another cable. |
| **0.96" OLED, I2C, 4 pins** (GND VCC SCL SDA) | SSD1306 128×64. Address 0x3C or 0x3D is found automatically. |
| **4 push buttons** (6×6 mm tactile) | UP, DOWN, SELECT, BACK. No resistors needed: the board's internal pull-ups are used. |
| Breadboard + about 10 male-male jumper wires | |
| **Gamepad** (optional) | An Xbox-style USB/Bluetooth pad is the easiest; PlayStation pads also work in Edge/Chrome. The keyboard works too. |
| USB-TTL adapter (optional, ~₹150) | CP2102 / CH340 / FTDI. **The R4 does NOT need it.** Only useful on event day if you want live plots while the organisers' program occupies the USB port (docs/09). |

**No flight sensors are needed.** In hardware-in-the-loop the simulator sends IMU, barometer, compass and GPS data to the Arduino over USB, and the Arduino sends the four motor commands back.

## Software (Windows)

| Software | Status on your PC |
|---|---|
| Arduino IDE 2 + "Arduino UNO R4 Boards" (1.6.0) | already installed |
| Microsoft Edge or Google Chrome | already installed. Needed because they support **Web Serial** (USB from a web page) and gamepads. Firefox will not work for "Connect Arduino". |
| Python 3 | already installed. `START_SIMULATOR.bat` uses it as a tiny local web server. |

## Software for the real Gazebo simulator (before the event)

The organisers' simulator needs **Ubuntu 22.04 + Gazebo Harmonic**.
- Your WSL has **Ubuntu 26.04**, which Gazebo Harmonic doesn't support.
- So you add a second WSL distribution, Ubuntu 22.04.
- You also need **usbipd-win** so Ubuntu can see the Arduino.

Everything is scripted in [08_gazebo_wsl_setup.md](08_gazebo_wsl_setup.md).

## What the simulator expects from the flight controller

From the organisers' `HIL_PROTOCOL.md` and `hil_bridge_v1.py`:

- USB serial at 115200 (the R4's native USB ignores the number), packets `"HI"` + header + payload + CRC32.
- Answer **HELLO** with **HELLO_ACK "UAV_HIL_V1"**. After **START**, send 4 motor values (0..1) back for every sensor burst (about 50 per second), plus a **HEARTBEAT** every 200 ms.
- If the bridge hears nothing for 1.5 s, it stops the motors.

The firmware does all of this. On START it arms and takes off to 1.5 m by itself (you can switch this off in the menu: Features → Auto TO).
