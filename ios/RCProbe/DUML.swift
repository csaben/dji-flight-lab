import Foundation

/// DJI's DUML v1 frame and the "55 CC" link envelope the remote expects from a phone over the cable.
/// Layout and CRCs as documented by the community (dji-firmware-tools, samuelsadok/dji_protocol, and the
/// RC-N3 iOS notes); this is our own implementation.
enum DUML {
    // Module ids (bits 0-4 type, 5-7 index)
    static let camera: UInt8 = 0x01, app: UInt8 = 0x02, flightController: UInt8 = 0x03, gimbal: UInt8 = 0x04
    static let remote: UInt8 = 0x06, wifi: UInt8 = 0x07, hdlinkAir: UInt8 = 0x09, hdlinkGround: UInt8 = 0x0E

    static let names: [UInt8: String] = [0x00: "any", 0x01: "camera", 0x02: "app", 0x03: "flight", 0x04: "gimbal", 0x05: "center",
        0x06: "remote", 0x07: "wifi", 0x08: "dm368", 0x09: "link-air", 0x0A: "pc", 0x0B: "battery", 0x0C: "esc",
        0x0D: "dm368-g", 0x0E: "link-gnd", 0x11: "mvo", 0x12: "visual", 0x1B: "rtk"]
    static func name(_ m: UInt8) -> String { (names[m & 0x1F] ?? String(format: "m%02x", m & 0x1F)) + (m >> 5 > 0 ? "\(m >> 5)" : "") }

    static let crc8Table: [UInt8] = (0..<256).map { i in
        var c = UInt8(i); for _ in 0..<8 { c = c & 1 != 0 ? (c >> 1) ^ 0x8C : c >> 1 }; return c }
    static let crc16Table: [UInt16] = (0..<256).map { i in
        var c = UInt16(i); for _ in 0..<8 { c = c & 1 != 0 ? (c >> 1) ^ 0x8408 : c >> 1 }; return c }

    static func crc8<C: Collection>(_ b: C) -> UInt8 where C.Element == UInt8 {
        var c: UInt8 = 0x77; for x in b { c = crc8Table[Int(c ^ x)] }; return c }
    static func crc16<C: Collection>(_ b: C) -> UInt16 where C.Element == UInt8 {
        var v: UInt16 = 0x3692; for x in b { v = (v >> 8) ^ crc16Table[Int((v ^ UInt16(x)) & 0xFF)] }; return v }

    struct Frame {
        var sender: UInt8, receiver: UInt8, seq: UInt16, type: UInt8, set: UInt8, id: UInt8, payload: [UInt8]
        var isReply: Bool { type & 0x80 != 0 }
        var key: String { "\(DUML.name(sender))→\(DUML.name(receiver)) \(String(format: "%02x/%02x", set, id))\(isReply ? " r" : "")" }
    }

    static func build(_ f: Frame) -> [UInt8] {
        let len = 13 + f.payload.count
        var b: [UInt8] = [0x55, UInt8(len & 0xFF), UInt8((len >> 8) & 0x03) | 0x04]
        b.append(crc8(b))
        b += [f.sender, f.receiver, UInt8(f.seq & 0xFF), UInt8(f.seq >> 8), f.type, f.set, f.id] + f.payload
        let c = crc16(b); b += [UInt8(c & 0xFF), UInt8(c >> 8)]
        return b
    }

    /// Parse one frame at the start of `b`; returns the frame and its length, nil if not a valid frame.
    static func parse(_ b: ArraySlice<UInt8>) -> (Frame, Int)? {
        let s = b.startIndex
        guard b.count >= 13, b[s] == 0x55 else { return nil }
        let len = Int(b[s + 1]) | (Int(b[s + 2] & 0x03) << 8)
        guard len >= 13, crc8(b[s..<s + 3]) == b[s + 3] else { return nil }
        guard b.count >= len else { return nil }
        let c = crc16(b[s..<s + len - 2])
        guard UInt8(c & 0xFF) == b[s + len - 2], UInt8(c >> 8) == b[s + len - 1] else { return nil }
        return (Frame(sender: b[s + 4], receiver: b[s + 5], seq: UInt16(b[s + 6]) | UInt16(b[s + 7]) << 8,
                      type: b[s + 8], set: b[s + 9], id: b[s + 10], payload: Array(b[s + 11..<s + len - 2])), len)
    }

    static let dumlPort: UInt16 = 22345, dumlPortIn: UInt16 = 30000, videoPort: UInt16 = 22346

    static func envelope(_ payload: [UInt8], port: UInt16 = dumlPort) -> [UInt8] {
        let n = UInt32(payload.count)
        return [0x55, 0xCC, UInt8(port & 0xFF), UInt8(port >> 8),
                UInt8(n & 0xFF), UInt8((n >> 8) & 0xFF), UInt8((n >> 16) & 0xFF), UInt8(n >> 24)] + payload
    }
}
