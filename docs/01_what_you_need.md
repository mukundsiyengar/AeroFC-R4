# 1. What you need

## Hardware

| Item | Notes |
|---|---|
| **Arduino UNO R4 Minima** | The flight controller. It talks to the PC over its USB-C port (native USB). Other boards: [docs/16](16_other_boards.md). |
| **USB-C cable that carries data** | Some cables only charge. If Windows doesn't make its connect sound when you plug the board in, try another cable. |
| **0.96" OLED, I2C, 4 pins** | SSD1306, 128 x 64. Address 0x3C or 0x3D is found automatically. Optional: it flies without one. |
| **4 push buttons** (6 x 6 mm tactile) | UP, DOWN, SELECT, BACK. No resistors needed: the board's internal pull-ups are used. Optional. |
| Breadboard and about 10 jumper wires | |
| **Gamepad** (optional) | An Xbox-style USB or Bluetooth pad is the easiest, and PlayStation pads also work in Edge or Chrome. The keyboard works too. This hasn't been tested with a real pad, only in the page's test suite. |
| USB-TTL adapter (optional) | CP2102, CH340 or FTDI. **The R4 doesn't need it.** It's only useful if you want live plots while another program is using the USB port ([docs/09](09_event_day_checklist.md)). |

**No flight sensors are needed.** In hardware-in-the-loop the simulator sends IMU, barometer, compass and GPS data to the Arduino over USB, and the Arduino sends the four motor commands back.

## Software on Windows

| Software | Why |
|---|---|
| Arduino IDE 2 with "Arduino UNO R4 Boards" | To build and upload the firmware |
| Microsoft Edge or Google Chrome | They support **Web Serial** (USB from a web page) and gamepads. Firefox won't work for "Connect Arduino". |
| Python 3 | `START_SIMULATOR.bat` uses it as a small local web server. Nothing to install with pip for flying. |

## Software for the real Gazebo simulator

The simulator needs **Ubuntu 22.04 and Gazebo Harmonic**.

- On Windows that means a WSL distribution called `Ubuntu-22.04`, and **usbipd-win** so Ubuntu can see the Arduino.
- On a Linux PC you only need Ubuntu 22.04.

Everything is scripted in [docs/08](08_gazebo_wsl_setup.md).

## Trying it with nothing at all

`START_SIMULATOR.bat`, then **Fly the software FC**, flies the real flight code in a browser replica of the drone, with no board and no Gazebo. It's a good way to see what the project does.

## What the simulator expects from the flight controller

From the simulator's `HIL_PROTOCOL.md` and `hil_bridge_v1.py`:

- USB serial at 115200 (the R4's native USB ignores the number), packets `"HI"` + header + payload + CRC32.
- Answer **HELLO** with **HELLO_ACK "UAV_HIL_V1"**. After **START**, send 4 motor values (0 to 1) back for every sensor burst (about 50 per second), plus a **HEARTBEAT** every 200 ms.
- If the bridge hears nothing for 1.5 s, it stops the motors.

The firmware does all of this. On START it arms and takes off to 1.5 m by itself. You can switch this off in the menu: Features > Auto TO.
