import SwiftUI

@main
struct RCProbeApp: App {
    @StateObject var link = Link()
    var body: some Scene { WindowGroup { Probe().environmentObject(link) } }
}

struct Probe: View {
    @EnvironmentObject var link: Link
    @State var share = false

    var body: some View {
        NavigationStack {
            List {
                Section("Remote") {
                    Text(link.status).font(.headline)
                    Text(link.accessory).font(.caption.monospaced())
                }
                Section("Sticks (364 … 1024 … 1684)") {
                    ForEach(Array(zip(["Right X", "Right Y", "Left Y", "Left X", "Gimbal"], link.sticks)), id: \.0) { n, v in
                        HStack {
                            Text(n).frame(width: 64, alignment: .leading)
                            ProgressView(value: Double(min(max(v, 364), 1684) - 364), total: 1320)
                            Text("\(v)").monospacedDigit().frame(width: 44, alignment: .trailing)
                        }.font(.caption)
                    }
                }
                Section("Who answered (version)") {
                    if link.versions.isEmpty { Text("Nobody yet").foregroundStyle(.secondary) }
                    ForEach(link.versions.sorted { $0.key < $1.key }, id: \.key) { k, v in
                        HStack { Text(k); Spacer(); Text(v).font(.caption.monospaced()) }
                    }
                }
                Section("Traffic") {
                    Text(link.stats).font(.caption2.monospaced())
                    ForEach(link.table, id: \.0) { k, n in
                        HStack { Text(k).font(.caption2.monospaced()); Spacer(); Text("\(n)").font(.caption2.monospaced()) }
                    }
                }
                Section("Log") {
                    ForEach(Array(link.lines.suffix(80).enumerated()), id: \.offset) { _, l in Text(l).font(.caption2.monospaced()) }
                }
            }
            .navigationTitle("RC Probe")
            .toolbar { Button("Share log") { share = true } }
            .sheet(isPresented: $share) { ShareSheet(items: [link.logFile]) }
        }
    }
}

struct ShareSheet: UIViewControllerRepresentable {
    let items: [Any]
    func makeUIViewController(context: Context) -> UIActivityViewController { UIActivityViewController(activityItems: items, applicationActivities: nil) }
    func updateUIViewController(_ c: UIActivityViewController, context: Context) {}
}
