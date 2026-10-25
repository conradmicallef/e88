import Foundation
import Network

/// Owns the UDP socket and the 30 ms control loop.
/// UI writes stick values; the loop reads them on its own queue.
final class DroneLink: ObservableObject {
    enum State: Equatable { case idle, connecting, ready, failed(String) }

    @Published private(set) var state: State = .idle
    @Published private(set) var packetsSent = 0
    @Published private(set) var packetsReceived = 0
    @Published private(set) var lastSentHex = ""
    @Published private(set) var lastReceivedHex = ""
    @Published private(set) var lastReceivedAt: Date?

    private let queue = DispatchQueue(label: "e88.link")
    private let lock = NSLock()
    private var connection: NWConnection?
    private var timer: DispatchSourceTimer?

    // Shared between UI and loop, guarded by `lock`.
    private var sticks = Sticks()
    private var pulseFlags: DroneFlags = []
    private var pulseUntil = Date.distantPast
    private var headless = false
    private var trims = Trims()
    private var profile = DroneProfile.e88Pro

    // Loop-only state.
    private var lastHeartbeat = Date.distantPast
    private var sentSinceUIUpdate = 0
    private var lastUIUpdate = Date.distantPast

    static let interval: TimeInterval = 0.03

    // MARK: - Public API (call from main thread)

    func connect(profile: DroneProfile, trims: Trims) {
        disconnect()
        locked { self.profile = profile; self.trims = trims; self.sticks = Sticks() }
        state = .connecting

        let params = NWParameters.udp
        params.requiredInterfaceType = .wifi
        guard let port = NWEndpoint.Port(rawValue: profile.port) else {
            state = .failed("Bad port"); return
        }
        let conn = NWConnection(host: NWEndpoint.Host(profile.host), port: port, using: params)
        conn.stateUpdateHandler = { [weak self, weak conn] s in
            DispatchQueue.main.async {
                guard let self, let conn, self.connection === conn else { return }
                self.handle(s)
            }
        }
        connection = conn
        conn.start(queue: queue)
        receive(on: conn)

        queue.async { [weak self] in
            guard let self else { return }
            self.send(DroneProtocol.heartbeat)
            self.startLoop()
        }
    }

    func disconnect() {
        let t = timer, c = connection
        timer = nil; connection = nil
        queue.async {
            t?.cancel()
            // Official apps send 08 01 when leaving flight control.
            c?.send(content: Data(DroneProtocol.leaveControl), completion: .contentProcessed { _ in c?.cancel() })
        }
        state = .idle
    }

    func setSticks(_ s: Sticks) { locked { sticks = s } }
    func setHeadless(_ on: Bool) { locked { headless = on } }
    func setTrims(_ t: Trims) { locked { trims = t } }

    /// Hold a momentary command (takeoff, land, stop, flip, calibrate) for `duration`.
    func pulse(_ flags: DroneFlags, duration: TimeInterval = 1.0) {
        locked {
            pulseFlags = flags
            pulseUntil = Date().addingTimeInterval(duration)
            if flags.contains(.stop) || flags.contains(.land) { sticks = Sticks() }
        }
    }

    // MARK: - Loop

    private func startLoop() {
        let t = DispatchSource.makeTimerSource(queue: queue)
        t.schedule(deadline: .now(), repeating: Self.interval, leeway: .milliseconds(3))
        t.setEventHandler { [weak self] in self?.tick() }
        timer = t
        t.resume()
    }

    private func tick() {
        let now = Date()
        var s = Sticks(), f: DroneFlags = [], tr = Trims(), p = DroneProfile.e88Pro
        locked {
            s = sticks; tr = trims; p = profile
            if now < pulseUntil { f.formUnion(pulseFlags) } else { pulseFlags = [] }
            if headless { f.insert(.headless) }
        }

        let packet = DroneProtocol.controlPacket(s, flags: f, trims: tr, profile: p)
        send(packet)

        if p.sendHeartbeat, now.timeIntervalSince(lastHeartbeat) >= 1 {
            lastHeartbeat = now
            send(DroneProtocol.heartbeat)
        }

        sentSinceUIUpdate += 1
        if now.timeIntervalSince(lastUIUpdate) > 0.25 {
            lastUIUpdate = now
            let n = sentSinceUIUpdate, hex = DroneProtocol.hex(packet)
            sentSinceUIUpdate = 0
            DispatchQueue.main.async {
                self.packetsSent += n
                self.lastSentHex = hex
            }
        }
    }

    private func send(_ bytes: [UInt8]) {
        connection?.send(content: Data(bytes), completion: .contentProcessed { _ in })
    }

    private func receive(on conn: NWConnection) {
        conn.receiveMessage { [weak self] data, _, _, error in
            guard let self else { return }
            if let data, !data.isEmpty {
                let hex = DroneProtocol.hex(Array(data.prefix(32)))
                DispatchQueue.main.async {
                    self.packetsReceived += 1
                    self.lastReceivedHex = hex
                    self.lastReceivedAt = Date()
                }
            }
            if error == nil, self.connection === conn { self.receive(on: conn) }
        }
    }

    private func handle(_ s: NWConnection.State) {
        switch s {
        case .ready: state = .ready
        case .failed(let e): state = .failed(e.localizedDescription)
        case .waiting(let e): state = .failed("Waiting: \(e.localizedDescription)")
        case .cancelled: if state != .idle { state = .idle }
        default: break
        }
    }

    private func locked(_ body: () -> Void) {
        lock.lock(); body(); lock.unlock()
    }
}
