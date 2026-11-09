import Foundation
import ExternalAccessory
import UIKit

/// Talks to a DJI remote plugged into the iPhone. Read-only on purpose: it only asks modules for their version,
/// reads the sticks and sends the link keepalive. It never sends a flight, camera or settings command.
final class Link: NSObject, ObservableObject, StreamDelegate {
    @Published var accessory = "None. Plug the iPhone into the remote's phone cable."
    @Published var status = "Waiting for the remote"
    @Published var sticks = [1024, 1024, 1024, 1024, 1024]
    @Published var table: [(String, Int)] = []
    @Published var versions: [String: String] = [:]
    @Published var stats = ""
    @Published var lines: [String] = []

    private var session: EASession?
    private var inBuf = [UInt8]()
    private var outQ = [UInt8]()
    private var seq: UInt16 = 1
    private var counts: [String: Int] = [:]
    private var ports: [UInt16: Int] = [:]
    private var bytesIn = 0, envelopes = 0, frames = 0, junk = 0, videoBytes = 0
    private var timers: [Timer] = []
    private var lastStick = Date.distantPast
    private let logURL: URL
    private var logH: FileHandle?
    private let t0 = Date()

    override init() {
        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let f = DateFormatter(); f.dateFormat = "yyyyMMdd-HHmmss"
        logURL = docs.appendingPathComponent("rcprobe-\(f.string(from: Date())).log")
        super.init()
        FileManager.default.createFile(atPath: logURL.path, contents: nil)
        logH = try? FileHandle(forWritingTo: logURL)
        log("RC Probe \(UIDevice.current.systemName) \(UIDevice.current.systemVersion) \(UIDevice.current.model)")
        NotificationCenter.default.addObserver(self, selector: #selector(changed), name: .EAAccessoryDidConnect, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(changed), name: .EAAccessoryDidDisconnect, object: nil)
        EAAccessoryManager.shared().registerForLocalNotifications()
        changed()
        timers.append(Timer.scheduledTimer(withTimeInterval: 0.25, repeats: true) { [weak self] _ in self?.publish() })
    }

    var logFile: URL { logH?.synchronizeFile(); return logURL }

    func log(_ s: String) {
        let line = String(format: "%7.2f ", Date().timeIntervalSince(t0)) + s
        logH?.write((line + "\n").data(using: .utf8)!)
        DispatchQueue.main.async { self.lines.append(line); if self.lines.count > 300 { self.lines.removeFirst(100) } }
    }

    // MARK: accessory

    @objc private func changed() {
        let all = EAAccessoryManager.shared().connectedAccessories
        for a in all {
            log("accessory: name=\(a.name) maker=\(a.manufacturer) model=\(a.modelNumber) fw=\(a.firmwareRevision) hw=\(a.hardwareRevision) protocols=\(a.protocolStrings)")
        }
        guard let a = all.first(where: { $0.manufacturer.lowercased().contains("dji") || $0.protocolStrings.contains { $0.hasPrefix("com.dji") } }) else {
            if session != nil { log("remote unplugged"); close() }
            accessory = all.isEmpty ? "None. Plug the iPhone into the remote's phone cable." : "Only non-DJI accessories: \(all.map(\.name))"
            status = "Waiting for the remote"
            return
        }
        accessory = "\(a.name) · \(a.modelNumber) · fw \(a.firmwareRevision)\n\(a.protocolStrings.joined(separator: ", "))"
        if session == nil { open(a) }
    }

    private func open(_ a: EAAccessory) {
        let order = ["com.dji.logiclink", "com.dji.protocol", "com.dji.common", "com.dji.video", "com.dji.fly"]
        for p in order where a.protocolStrings.contains(p) {
            guard let s = EASession(accessory: a, forProtocol: p) else { log("EASession \(p): refused"); continue }
            session = s
            for st in [s.inputStream, s.outputStream] as [Stream?] {
                st?.delegate = self; st?.schedule(in: .main, forMode: .default); st?.open()
            }
            status = "Connected over \(p)"
            log("EASession open on \(p)")
            start()
            return
        }
        status = "Remote found, but iOS would not open a session (\(a.protocolStrings))"
        log(status)
    }

    private func close() {
        timers.dropFirst().forEach { $0.invalidate() }; timers = Array(timers.prefix(1))
        for st in [session?.inputStream, session?.outputStream] as [Stream?] { st?.close(); st?.remove(from: .main, forMode: .default) }
        session = nil; inBuf.removeAll(); outQ.removeAll()
    }

    // MARK: what we send (read-only requests)

    private func send(_ to: UInt8, _ set: UInt8, _ id: UInt8, _ payload: [UInt8] = [], quiet: Bool = false) {
        seq &+= 1
        let f = DUML.Frame(sender: DUML.app, receiver: to, seq: seq, type: 0x40, set: set, id: id, payload: payload)
        outQ += DUML.envelope(DUML.build(f))
        if !quiet { log("→ \(f.key) seq=\(seq)") }
        pump()
    }

    private func start() {
        // Who is out there: version requests to the remote, the radio, and (if the drone is on) the aircraft modules.
        let modules: [UInt8] = [DUML.remote, DUML.hdlinkGround, DUML.hdlinkAir, DUML.flightController, DUML.gimbal, DUML.camera, 0x0B]
        for (i, m) in modules.enumerated() {
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.3 + Double(i) * 0.15) { self.send(m, 0x00, 0x01) }
        }
        // Link keepalive (the remote drops a quiet phone link), as both iOS tools found.
        timers.append(Timer.scheduledTimer(withTimeInterval: 2.5, repeats: true) { [weak self] _ in
            for d in [DUML.remote, DUML.hdlinkGround] { self?.send(d, 0x06, 0x77, [0x01, 0x01, 0x00, 0xFF, 0xFF, 0x20, 0x00, 0x00], quiet: true) }
        })
        // Stick positions, 10 Hz.
        timers.append(Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in self?.send(DUML.remote, 0x06, 0x01, quiet: true) })
        // Repeat the version sweep every 10 s, so turning the drone on later still shows up.
        timers.append(Timer.scheduledTimer(withTimeInterval: 10, repeats: true) { [weak self] _ in
            for m in modules { self?.send(m, 0x00, 0x01, quiet: true) }
        })
    }

    private func pump() {
        guard let o = session?.outputStream, o.hasSpaceAvailable, !outQ.isEmpty else { return }
        let n = outQ.withUnsafeBufferPointer { o.write($0.baseAddress!, maxLength: $0.count) }
        if n > 0 { outQ.removeFirst(n) } else if n < 0 { log("write error \(o.streamError?.localizedDescription ?? "")") }
    }

    func stream(_ s: Stream, handle e: Stream.Event) {
        switch e {
        case .hasBytesAvailable:
            guard let i = s as? InputStream else { return }
            var buf = [UInt8](repeating: 0, count: 65536)
            while i.hasBytesAvailable {
                let n = i.read(&buf, maxLength: buf.count)
                if n <= 0 { break }
                bytesIn += n; inBuf += buf[0..<n]
            }
            parse()
        case .hasSpaceAvailable: pump()
        case .errorOccurred: log("stream error: \(s.streamError?.localizedDescription ?? "?")")
        case .endEncountered: log("stream ended"); close()
        default: break
        }
    }

    // MARK: what comes back

    private func parse() {
        var i = 0
        let b = inBuf
        while i < b.count {
            if b[i] == 0x55, i + 1 < b.count, b[i + 1] == 0xCC {
                guard i + 8 <= b.count else { break }
                let port = UInt16(b[i + 2]) | UInt16(b[i + 3]) << 8
                let len = Int(b[i + 4]) | Int(b[i + 5]) << 8 | Int(b[i + 6]) << 16 | Int(b[i + 7]) << 24
                if len > 4_000_000 { i += 1; junk += 1; continue }
                guard i + 8 + len <= b.count else { break }
                envelopes += 1; ports[port, default: 0] += 1
                let body = b[(i + 8)..<(i + 8 + len)]
                if port == DUML.videoPort { videoBytes += len } else { frames(in: body) }
                i += 8 + len
            } else if b[i] == 0x55, let (f, n) = DUML.parse(b[i...]) {   // bare DUML (no envelope)
                handle(f); i += n
            } else if b[i] == 0x55, b.count - i < 1100 {
                break   // maybe a frame that isn't complete yet
            } else {
                i += 1; if b[i - 1] != 0 { junk += 1 }
            }
        }
        inBuf.removeFirst(i)
        if inBuf.count > 8_000_000 { inBuf.removeAll(); log("input buffer overflow, cleared") }
    }

    private func frames(in body: ArraySlice<UInt8>) {
        var j = body.startIndex
        while j < body.endIndex {
            if body[j] == 0x55, let (f, n) = DUML.parse(body[j...]) { handle(f); j += n } else { j += 1 }
        }
    }

    private func handle(_ f: DUML.Frame) {
        frames += 1
        counts[f.key, default: 0] += 1
        if counts[f.key] == 1 {
            log("← new: \(f.key) len=\(f.payload.count) \(f.payload.prefix(24).map { String(format: "%02x", $0) }.joined())")
        }
        if f.set == 0x00, f.id == 0x01, f.isReply, f.payload.count >= 18 {
            let hw = String(bytes: f.payload[2..<18].prefix { $0 != 0 }, encoding: .ascii) ?? "?"
            var fw = ""
            if f.payload.count >= 26 { let p = Array(f.payload[22..<26]); fw = " fw \(p[3]).\(p[2]).\(p[1]).\(p[0])" }
            let v = "\(hw)\(fw)"
            if versions[DUML.name(f.sender)] != v {
                log("version \(DUML.name(f.sender)): \(v)")
                DispatchQueue.main.async { self.versions[DUML.name(f.sender)] = v }
            }
        }
        // Sticks: 06/01 reply (status + 8 × (tag, LE16)) or 06/05 push (LE16 × 5).
        if f.set == 0x06, f.id == 0x01, f.isReply, f.payload.count >= 16 {
            let p = f.payload; sticksFrom((0..<5).map { Int(p[2 + 3 * $0]) | Int(p[3 + 3 * $0]) << 8 })
        } else if f.set == 0x06, f.id == 0x05, f.payload.count >= 10 {
            let p = f.payload; sticksFrom((0..<5).map { Int(p[2 * $0]) | Int(p[2 * $0 + 1]) << 8 })
        }
    }

    private func sticksFrom(_ v: [Int]) {
        guard v.allSatisfy({ (300...1750).contains($0) }) else { return }
        let now = Date()
        if now.timeIntervalSince(lastStick) > 0.05 { lastStick = now; DispatchQueue.main.async { self.sticks = v } }
    }

    private func publish() {
        table = counts.sorted { $0.value > $1.value }.prefix(40).map { ($0.key, $0.value) }
        stats = "in \(bytesIn / 1024) KB · envelopes \(envelopes) · DUML frames \(frames) · video \(videoBytes / 1024) KB · junk \(junk)\nports " +
            ports.sorted { $0.key < $1.key }.map { "\($0.key):\($0.value)" }.joined(separator: " ")
    }
}
