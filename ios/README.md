# E88 Controller

An iPhone controller for E88 / E88 Pro style Wi-Fi toy drones. It has two virtual sticks, take off / land / flip / emergency stop, headless mode, speed modes, trims and gyro calibration. A diagnostics panel shows every packet sent and anything the drone sends back.

It is built entirely on GitHub Actions and delivered through TestFlight, so no Mac is needed.

## Protocol

There are two presets, picked automatically from the phone's Wi-Fi address:

| Preset | Drone IP | UDP port | Packet |
|---|---|---|---|
| E88 Pro (RC-UFO app) | 192.168.1.1 | 7099 | `03 66 RR PP TT YY FF CS 99` + `01 01` heartbeat each second |
| WiFi UFO (E58 style) | 192.168.0.1 | 50000 | `66 RR PP TT YY FF CS 99` |

- `RR/PP/TT/YY` are roll, pitch, throttle and yaw, with 128 as centre (1…255).
- `CS` is the XOR of the five bytes before it.
- `FF` holds the flags: takeoff `0x01`, land `0x02`, stop `0x04`, flip `0x08`, headless `0x10`, calibrate `0x80`.
- Packets are sent every 30 ms.

All of these can be edited in Settings if your clone differs.

## One-time setup (all doable from a phone browser)

1. **Register the bundle ID.** At developer.apple.com go to Certificates, IDs & Profiles → Identifiers → **+** → App IDs → App. Use the explicit bundle ID `com.conradmicallef.e88controller`.
2. **Create the app record.** At appstoreconnect.apple.com go to Apps → **+** → New App. Choose platform iOS, name it "E88 Controller" (any unique name), and pick the bundle ID above. The SKU can be anything, e.g. `e88`.
3. **Create an API key.** In App Store Connect go to Users and Access → Integrations → App Store Connect API → **+**. Give it **Admin** access (needed for cloud signing). Download the `.p8` file, which can only be downloaded once, and note the **Key ID** and the **Issuer ID** shown above the list.
4. **Find your Team ID.** It's shown at developer.apple.com → Account → Membership details.
5. **Add GitHub secrets.** In this repo go to Settings → Secrets and variables → Actions → New repository secret, and add:
   - `TEAM_ID`
   - `ASC_KEY_ID`
   - `ASC_ISSUER_ID`
   - `ASC_KEY_P8`: paste the whole contents of the `.p8` file, including the BEGIN/END lines
6. **Run the build.** Go to Actions → *Build & TestFlight* → Run workflow. After Apple processes the build (5–20 min), install the **TestFlight** app on your iPhone and the build will appear there. You're automatically an internal tester as the account holder.

Every push to `main` builds and uploads a new TestFlight build.

## Flying

1. Power on the drone and join its Wi-Fi in iOS Settings.
2. Open the app, tap **Connect**, and allow *Local Network* access when asked.
3. **Take off**, then fly. The left stick is throttle/yaw and the right stick is pitch/roll (Mode 2). Throttle centred means hold altitude.
4. **STOP MOTORS** cuts the motors instantly. Use it only in an emergency, because the drone will fall.

Leaving the app while flying sends a land command.

If the status stays on **Sending** and never changes to **Drone replying**, and the drone doesn't respond, open Settings → Diagnostics. Try the other preset, or toggle the prefix and checksum options.
