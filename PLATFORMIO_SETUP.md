# PlatformIO + Wokwi setup

This package is laid out as a standard PlatformIO project.

- `src/main.cpp` — integrated Dat hardware + Snay cloud code
- `include/cloud_config.h` — Blynk template/broker config
- `include/secrets.h` — local Wi-Fi/Auth Token only; do not commit
- `diagram.json` — Dat's full Wokwi circuit
- `wokwi.toml` — points Wokwi to PlatformIO firmware/ELF

## Run
1. Put the test Blynk Auth Token in `include/secrets.h`.
2. PlatformIO: Build.
3. `F1` → `Wokwi: Start Simulator`.
4. Verify serial output for Wi-Fi, NTP, MQTT, and HTTPS cloud heartbeat.
