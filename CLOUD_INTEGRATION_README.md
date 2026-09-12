# Dat + Snay Smart Parking Integration

This package preserves Dat's six-ultrasonic / two-pressure-plate / servo / LED / buzzer controller and adds Snay's proven Blynk cloud communications.

## Cloud mapping

- V0 `Bay1Occupied` — MQTT/TLS
- V1 `Bay2Occupied` — MQTT/TLS
- V2 `AvailableSpaces` — MQTT/TLS
- V3 `GateState` — MQTT/TLS
- V4 `Bay1DurationSec` — MQTT/TLS
- V5 `Bay2DurationSec` — MQTT/TLS
- V6 `EntryPlate` — MQTT/TLS
- V7 `ExitPlate` — MQTT/TLS
- V8 `FullCapacity` — HTTPS/TLS
- V9 `Bay1Overstay` — HTTPS/TLS
- V10 `Bay2Overstay` — HTTPS/TLS
- V11 `SensorFault` — HTTPS/TLS; currently fixed false until Aasman's fault detector is integrated
- V12 `WifiRSSI` — HTTPS/TLS
- V13 `SystemStatus` — HTTPS/TLS
- V14 `AlertMessage` — HTTPS/TLS

## Gate state values

- 0 = CLOSED
- 1 = OPEN_ENTRY
- 2 = OPEN_EXIT
- 3 = WAITING_TO_CLOSE
- 4 = LOCKED_FULL

## Before running

1. Put the Blynk device Auth Token in `secrets.h`.
2. Do not commit `secrets.h`; `.gitignore` already excludes it.
3. The Blynk template ID and region are in `cloud_config.h`.
4. `libraries.txt` now includes ESP32Servo, PubSubClient, ArduinoJson and Blynk.

## Design notes

- Dat/Aasman's controller remains authoritative for occupancy, overstay, capacity, plates and gate logic.
- Snay's module only reports controller state to the cloud.
- Important state changes publish immediately.
- A full heartbeat is sent every 30 seconds for history/re-synchronisation.
- Wi-Fi/MQTT reconnect attempts use `millis()` rather than long reconnect loops.
- HTTPS timeout is deliberately short (1.5 s) to reduce the effect of a cloud-side failure on local control.
- Dat's original gate hold remains 6000 ms in this handoff. Change it in the controller layer only if the team finalises the agreed 10–15 s requirement.

## Integration boundary still to complete

`SensorFault` remains `false` because Dat's current source does not yet contain Aasman's authoritative fault detector. When that logic is merged, assign its output in `syncTelemetryFromController()`.
