import Foundation

/// Connection + packet settings for one family of E88-style toy drones.
/// The clones differ in IP, port and a couple of framing details, so all of
/// it is editable in Settings.
struct DroneProfile: Codable, Equatable {
    var name: String
    var host: String
    var port: UInt16
    /// E88 Pro / RC-UFO app firmware wants a 0x03 byte before the 0x66 header.
    var usePrefix: Bool
    /// Send a 0x01 0x01 keep-alive once per second.
    var sendHeartbeat: Bool
    /// Fill the XOR checksum byte (some firmwares accept 0 here).
    var checksum: Bool

    static let e88Pro = DroneProfile(name: "E88 Pro (RC-UFO app)",
                                     host: "192.168.1.1", port: 7099,
                                     usePrefix: true, sendHeartbeat: true, checksum: true)
    static let wifiUFO = DroneProfile(name: "WiFi UFO (E58 style)",
                                      host: "192.168.0.1", port: 50000,
                                      usePrefix: false, sendHeartbeat: false, checksum: true)
    static let presets: [DroneProfile] = [e88Pro, wifiUFO]

    /// Pick a preset from the phone's Wi-Fi address (drone networks hand out
    /// addresses in the drone's own subnet).
    static func guess(forPhoneIP ip: String?) -> DroneProfile? {
        guard let ip else { return nil }
        if ip.hasPrefix("192.168.1.") { return .e88Pro }
        if ip.hasPrefix("192.168.0.") { return .wifiUFO }
        return nil
    }
}

struct DroneFlags: OptionSet {
    let rawValue: UInt8
    static let takeoff   = DroneFlags(rawValue: 0x01)
    static let land      = DroneFlags(rawValue: 0x02)
    static let stop      = DroneFlags(rawValue: 0x04)
    static let flip      = DroneFlags(rawValue: 0x08)
    static let headless  = DroneFlags(rawValue: 0x10)
    static let calibrate = DroneFlags(rawValue: 0x80)
}

/// Stick positions, each -1...1 (0 = centred). Throttle 0 = hold altitude.
struct Sticks: Equatable {
    var roll = 0.0, pitch = 0.0, throttle = 0.0, yaw = 0.0
}

struct Trims: Codable, Equatable {
    var roll = 0, pitch = 0, yaw = 0
}

enum DroneProtocol {
    static let header: UInt8 = 0x66
    static let footer: UInt8 = 0x99
    static let prefix: UInt8 = 0x03
    static let heartbeat: [UInt8] = [0x01, 0x01]
    /// Sent by the official apps when leaving flight control (not at startup).
    static let leaveControl: [UInt8] = [0x08, 0x01]

    /// -1 → 1, 0 → 128, +1 → 255, plus a trim offset in raw units.
    static func axisByte(_ value: Double, trim: Int = 0) -> UInt8 {
        let v = max(-1, min(1, value.isFinite ? value : 0))
        let raw = Int((128 + v * 127).rounded()) + trim
        return UInt8(max(1, min(255, raw)))
    }

    static func controlPacket(_ s: Sticks, flags: DroneFlags, trims: Trims,
                              profile: DroneProfile) -> [UInt8] {
        let r = axisByte(s.roll, trim: trims.roll)
        let p = axisByte(s.pitch, trim: trims.pitch)
        let t = axisByte(s.throttle)
        let y = axisByte(s.yaw, trim: trims.yaw)
        let f = flags.rawValue
        let cs: UInt8 = profile.checksum ? (r ^ p ^ t ^ y ^ f) : 0
        var packet: [UInt8] = [header, r, p, t, y, f, cs, footer]
        if profile.usePrefix { packet.insert(prefix, at: 0) }
        return packet
    }

    static func hex(_ bytes: [UInt8]) -> String {
        bytes.map { String(format: "%02X", $0) }.joined(separator: " ")
    }
}
