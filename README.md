# 3707ICT-Automated-Smart-Parking

## Reproduce the build

1. Install VS Code with the PlatformIO extension.
2. Clone this repository and open its folder in VS Code.
3. The repository includes `include/secrets.h` with the Wokwi test credentials, so the simulation is ready to run. Replace those values for physical hardware.
4. Run PlatformIO: Build. Dependencies are declared in `platformio.ini`.
5. For simulation, install the Wokwi VS Code extension and start the simulator using `diagram.json`.

The committed `include/secrets.h` contains only the dedicated Wokwi test token. Never replace it with a production Wi-Fi password or Blynk token before pushing changes.
