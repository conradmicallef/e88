import Foundation

enum SpeedMode: Int, CaseIterable, Codable {
    case slow = 0, normal, fast
    var label: String { ["Slow", "Normal", "Fast"][rawValue] }
    /// Scale applied to roll/pitch/yaw.
    var scale: Double { [0.4, 0.7, 1.0][rawValue] }
}

final class AppSettings: ObservableObject {
    @Published var profile: DroneProfile { didSet { save() } }
    @Published var trims: Trims { didSet { save() } }
    @Published var speed: SpeedMode { didSet { save() } }
    @Published var autoDetect: Bool { didSet { save() } }

    private let key = "e88.settings.v1"

    private struct Stored: Codable {
        var profile: DroneProfile
        var trims: Trims
        var speed: SpeedMode
        var autoDetect: Bool
    }

    init() {
        if let data = UserDefaults.standard.data(forKey: key),
           let s = try? JSONDecoder().decode(Stored.self, from: data) {
            profile = s.profile; trims = s.trims; speed = s.speed; autoDetect = s.autoDetect
        } else {
            profile = .e88Pro; trims = Trims(); speed = .normal; autoDetect = true
        }
    }

    /// Profile to use right now: auto-detected from the Wi-Fi subnet if enabled.
    func effectiveProfile() -> DroneProfile {
        if autoDetect, let g = DroneProfile.guess(forPhoneIP: NetInfo.wifiIPv4()) { return g }
        return profile
    }

    private func save() {
        let s = Stored(profile: profile, trims: trims, speed: speed, autoDetect: autoDetect)
        if let data = try? JSONEncoder().encode(s) { UserDefaults.standard.set(data, forKey: key) }
    }
}
