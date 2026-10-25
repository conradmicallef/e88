import SwiftUI
import UIKit

struct ContentView: View {
    @EnvironmentObject var link: DroneLink
    @EnvironmentObject var settings: AppSettings
    @Environment(\.scenePhase) private var scenePhase

    @State private var left = CGPoint.zero   // x = yaw, y = throttle
    @State private var right = CGPoint.zero  // x = roll, y = pitch
    @State private var headless = false
    @State private var showSettings = false
    @State private var now = Date()

    private let clock = Timer.publish(every: 0.5, on: .main, in: .common).autoconnect()

    var body: some View {
        ZStack {
            LinearGradient(colors: [Color(white: 0.09), Color(white: 0.03)],
                           startPoint: .top, endPoint: .bottom)
                .ignoresSafeArea()

            VStack(spacing: 0) {
                topBar
                Spacer(minLength: 8)
                HStack(alignment: .center) {
                    JoystickView(label: "THROTTLE · YAW") { left = $0; push() }
                    Spacer()
                    centerControls
                    Spacer()
                    JoystickView(label: "PITCH · ROLL") { right = $0; push() }
                }
                .padding(.horizontal, 12)
                Spacer(minLength: 16)
            }
            .padding(.vertical, 8)
        }
        .preferredColorScheme(.dark)
        .sheet(isPresented: $showSettings) {
            SettingsView().environmentObject(link).environmentObject(settings)
        }
        .onReceive(clock) { now = $0 }
        .onChange(of: headless) { link.setHeadless($0) }
        .onChange(of: settings.trims) { link.setTrims($0) }
        .onChange(of: link.state) { s in
            UIApplication.shared.isIdleTimerDisabled = (s == .ready)
        }
        .onChange(of: scenePhase) { phase in
            // Leaving the app mid-flight: tell the drone to land.
            if phase == .background, link.state == .ready {
                link.pulse(.land, duration: 2)
            }
        }
    }

    // MARK: - Pieces

    private var topBar: some View {
        HStack(spacing: 12) {
            statusPill
            Spacer()
            Picker("Speed", selection: $settings.speed) {
                ForEach(SpeedMode.allCases, id: \.self) { Text($0.label).tag($0) }
            }
            .pickerStyle(.segmented)
            .frame(width: 220)
            .onChange(of: settings.speed) { _ in push() }

            Toggle(isOn: $headless) { Text("Headless").font(.subheadline) }
                .toggleStyle(.button)
                .tint(headless ? .orange : .gray)

            Button { showSettings = true } label: {
                Image(systemName: "gearshape.fill").font(.title3)
            }
        }
        .padding(.horizontal, 20)
    }

    private var statusPill: some View {
        let replying = link.lastReceivedAt.map { now.timeIntervalSince($0) < 2 } ?? false
        let (text, color): (String, Color) = {
            switch link.state {
            case .idle: return ("Not connected", .gray)
            case .connecting: return ("Connecting…", .yellow)
            case .ready: return (replying ? "Drone replying" : "Sending", replying ? .green : .blue)
            case .failed(let m): return (m, .red)
            }
        }()
        return Button {
            if link.state == .idle || isFailed { link.connect(profile: settings.effectiveProfile(), trims: settings.trims) }
            else { link.disconnect() }
        } label: {
            HStack(spacing: 8) {
                Circle().fill(color).frame(width: 10, height: 10)
                Text(text).font(.subheadline.weight(.medium)).lineLimit(1)
                Text(link.state == .idle || isFailed ? "Connect" : "Disconnect")
                    .font(.caption.weight(.bold))
                    .padding(.horizontal, 8).padding(.vertical, 3)
                    .background(Capsule().fill(Color.white.opacity(0.15)))
            }
            .padding(.horizontal, 12).padding(.vertical, 7)
            .background(Capsule().fill(Color.white.opacity(0.08)))
        }
        .buttonStyle(.plain)
        .frame(maxWidth: 360, alignment: .leading)
    }

    private var isFailed: Bool { if case .failed = link.state { return true }; return false }

    private var centerControls: some View {
        VStack(spacing: 12) {
            HStack(spacing: 12) {
                actionButton("Take off", icon: "arrow.up.circle.fill", color: .green) {
                    link.pulse(.takeoff)
                }
                actionButton("Land", icon: "arrow.down.circle.fill", color: .blue) {
                    link.pulse(.land)
                }
            }
            actionButton("Flip", icon: "arrow.triangle.2.circlepath", color: .purple) {
                link.pulse(.flip, duration: 0.5)
            }
            Button {
                UINotificationFeedbackGenerator().notificationOccurred(.error)
                link.pulse(.stop, duration: 1.5)
            } label: {
                Label("STOP MOTORS", systemImage: "exclamationmark.octagon.fill")
                    .font(.headline)
                    .frame(width: 220, height: 50)
                    .background(RoundedRectangle(cornerRadius: 14).fill(Color.red))
                    .foregroundStyle(.white)
            }
        }
        .disabled(link.state != .ready)
        .opacity(link.state == .ready ? 1 : 0.4)
    }

    private func actionButton(_ title: String, icon: String, color: Color,
                              action: @escaping () -> Void) -> some View {
        Button {
            UIImpactFeedbackGenerator(style: .medium).impactOccurred()
            action()
        } label: {
            Label(title, systemImage: icon)
                .font(.subheadline.weight(.semibold))
                .frame(width: 104, height: 44)
                .background(RoundedRectangle(cornerRadius: 12).fill(color.opacity(0.85)))
                .foregroundStyle(.white)
        }
    }

    // MARK: - Mapping

    private func push() {
        let k = settings.speed.scale
        link.setSticks(Sticks(roll: dz(right.x) * k,
                              pitch: dz(right.y) * k,
                              throttle: dz(left.y),
                              yaw: dz(left.x) * k))
    }

    /// Small dead zone so a resting thumb doesn't drift the drone.
    private func dz(_ v: CGFloat) -> Double {
        let d = 0.06, x = Double(v)
        guard abs(x) > d else { return 0 }
        return (x - (x > 0 ? d : -d)) / (1 - d)
    }
}
