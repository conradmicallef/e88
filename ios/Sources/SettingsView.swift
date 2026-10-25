import SwiftUI

struct SettingsView: View {
    @EnvironmentObject var link: DroneLink
    @EnvironmentObject var settings: AppSettings
    @Environment(\.dismiss) private var dismiss

    @State private var portText = ""
    @State private var phoneIP = NetInfo.wifiIPv4()

    var body: some View {
        NavigationStack {
            Form {
                Section {
                    Toggle("Auto-detect from Wi-Fi", isOn: $settings.autoDetect)
                    LabeledContent("Phone address", value: phoneIP ?? "Not on Wi-Fi")
                    if settings.autoDetect {
                        LabeledContent("Will use", value: settings.effectiveProfile().name)
                    }
                } header: { Text("Drone") } footer: {
                    Text("Join the drone's Wi-Fi in iOS Settings first. Auto-detect picks a preset from your phone's address; turn it off to use the settings below. Reconnect after changing anything.")
                }

                Section("Manual profile") {
                    Picker("Preset", selection: presetBinding) {
                        ForEach(DroneProfile.presets, id: \.name) { Text($0.name).tag($0.name) }
                        Text("Custom").tag("Custom")
                    }
                    TextField("Drone IP", text: $settings.profile.host)
                        .keyboardType(.numbersAndPunctuation)
                        .autocorrectionDisabled()
                    TextField("UDP port", text: $portText)
                        .keyboardType(.numberPad)
                        .onChange(of: portText) { t in
                            if let p = UInt16(t) { settings.profile.port = p; markCustom() }
                        }
                    Toggle("0x03 prefix byte", isOn: customBinding(\.usePrefix))
                    Toggle("Heartbeat every second", isOn: customBinding(\.sendHeartbeat))
                    Toggle("XOR checksum", isOn: customBinding(\.checksum))
                }
                .disabled(settings.autoDetect)

                Section {
                    Stepper("Roll trim: \(settings.trims.roll)", value: $settings.trims.roll, in: -30...30)
                    Stepper("Pitch trim: \(settings.trims.pitch)", value: $settings.trims.pitch, in: -30...30)
                    Stepper("Yaw trim: \(settings.trims.yaw)", value: $settings.trims.yaw, in: -30...30)
                    Button("Reset trims") { settings.trims = Trims() }
                } header: { Text("Trim") } footer: {
                    Text("If the drone drifts while hovering hands-off, nudge the trim the opposite way.")
                }

                Section {
                    Button("Calibrate gyro") { link.pulse(.calibrate, duration: 1.5) }
                        .disabled(link.state != .ready)
                } footer: {
                    Text("Only with the drone sitting level on the ground, motors off.")
                }

                Section {
                    LabeledContent("Packets sent", value: "\(link.packetsSent)")
                    LabeledContent("Last sent", value: link.lastSentHex.isEmpty ? "—" : link.lastSentHex)
                        .font(.system(.footnote, design: .monospaced))
                    LabeledContent("Packets received", value: "\(link.packetsReceived)")
                    LabeledContent("Last received", value: link.lastReceivedHex.isEmpty ? "—" : link.lastReceivedHex)
                        .font(.system(.footnote, design: .monospaced))
                } header: { Text("Diagnostics") } footer: {
                    Text("Anything the drone sends back shows up here — useful for working out its protocol.")
                }
            }
            .navigationTitle("Settings")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar { ToolbarItem(placement: .confirmationAction) { Button("Done") { dismiss() } } }
            .onAppear {
                portText = String(settings.profile.port)
                phoneIP = NetInfo.wifiIPv4()
            }
        }
    }

    private var presetBinding: Binding<String> {
        Binding(
            get: { settings.profile.name },
            set: { name in
                if let p = DroneProfile.presets.first(where: { $0.name == name }) {
                    settings.profile = p
                    portText = String(p.port)
                } else {
                    settings.profile.name = "Custom"
                }
            })
    }

    private func customBinding(_ kp: WritableKeyPath<DroneProfile, Bool>) -> Binding<Bool> {
        Binding(get: { settings.profile[keyPath: kp] },
                set: { settings.profile[keyPath: kp] = $0; markCustom() })
    }

    private func markCustom() {
        if DroneProfile.presets.contains(settings.profile) { return }
        if settings.profile.name != "Custom" { settings.profile.name = "Custom" }
    }
}
