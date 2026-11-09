import SwiftUI
import WebKit

/// Flight Lab for iPhone: the web sandbox (dji.clarksaben.com) full screen. Not signed in -> the gateway's /login page.
/// No keys are built in; the login cookie lives in the app's own web data store.
let home = URL(string: "https://dji.clarksaben.com/")!

@main
struct FlightLabApp: App {
    var body: some Scene {
        WindowGroup {
            Web().ignoresSafeArea().background(Color(red: 0.04, green: 0.04, blue: 0.045)).statusBarHidden(true)
                .persistentSystemOverlays(.hidden)
        }
    }
}

struct Web: UIViewRepresentable {
    func makeCoordinator() -> Nav { Nav() }

    func makeUIView(context: Context) -> WKWebView {
        let c = WKWebViewConfiguration()
        c.allowsInlineMediaPlayback = true
        c.mediaTypesRequiringUserActionForPlayback = []
        c.websiteDataStore = .default()
        // The gateway's login page is unstyled (black on our dark view): give it the app's look.
        c.userContentController.addUserScript(WKUserScript(source: """
            if (location.pathname === '/login') { const s = document.createElement('style');
              s.textContent = 'body{background:#0a0a0b;color:#e8e8e8} *{box-sizing:border-box} input{border-radius:10px;border:1px solid #333;background:#16161a;color:#fff} button{border-radius:10px;border:0;background:#e8e8e8;color:#000}';
              document.head.appendChild(s); }
            """, injectionTime: .atDocumentEnd, forMainFrameOnly: true))
        let w = WKWebView(frame: .zero, configuration: c)
        w.navigationDelegate = context.coordinator
        w.uiDelegate = context.coordinator
        w.isOpaque = false
        w.backgroundColor = .black
        w.scrollView.contentInsetAdjustmentBehavior = .never
        w.scrollView.bounces = false
        w.allowsBackForwardNavigationGestures = false
        w.customUserAgent = nil
        w.configuration.applicationNameForUserAgent = "FlightLabiOS/1.0"
        let rc = UIRefreshControl()
        rc.addTarget(context.coordinator, action: #selector(Nav.refresh(_:)), for: .valueChanged)
        w.scrollView.refreshControl = rc
        context.coordinator.web = w
        w.load(URLRequest(url: home))
        return w
    }

    func updateUIView(_ w: WKWebView, context: Context) {}
}

final class Nav: NSObject, WKNavigationDelegate, WKUIDelegate {
    weak var web: WKWebView?

    @objc func refresh(_ rc: UIRefreshControl) { web?.reload(); rc.endRefreshing() }

    // Signed out (401 on a page): show the login form, which comes back to / when done.
    func webView(_ w: WKWebView, decidePolicyFor r: WKNavigationResponse, decisionHandler: @escaping (WKNavigationResponsePolicy) -> Void) {
        if r.isForMainFrame, let h = r.response as? HTTPURLResponse, h.statusCode == 401, h.url?.path != "/login" {
            decisionHandler(.cancel)
            w.load(URLRequest(url: URL(string: "/login", relativeTo: home)!))
            return
        }
        decisionHandler(.allow)
    }

    // Links to other sites open in Safari; everything on the gateway stays in the app.
    func webView(_ w: WKWebView, decidePolicyFor a: WKNavigationAction, decisionHandler: @escaping (WKNavigationActionPolicy) -> Void) {
        if let u = a.request.url, let host = u.host, host != home.host, a.targetFrame?.isMainFrame ?? true,
           u.scheme == "https" || u.scheme == "http" {
            UIApplication.shared.open(u)
            decisionHandler(.cancel)
            return
        }
        decisionHandler(.allow)
    }

    // target=_blank on the gateway: load in place.
    func webView(_ w: WKWebView, createWebViewWith c: WKWebViewConfiguration, for a: WKNavigationAction, windowFeatures f: WKWindowFeatures) -> WKWebView? {
        if let u = a.request.url { w.load(URLRequest(url: u)) }
        return nil
    }

    // Offline / tunnel down: a plain retry page instead of a blank screen.
    func webView(_ w: WKWebView, didFailProvisionalNavigation n: WKNavigation!, withError e: Error) {
        let ne = e as NSError
        // our own cancel (401 -> /login) or a newer load replacing this one: not an error
        if ne.code == NSURLErrorCancelled || (ne.domain == "WebKitErrorDomain" && ne.code == 102) { return }
        let msg = ne.localizedDescription.replacingOccurrences(of: "<", with: "&lt;")
        w.loadHTMLString("""
        <meta name=viewport content="width=device-width,initial-scale=1"><body style="background:#0a0a0b;color:#ddd;font:16px system-ui;display:flex;flex-direction:column;align-items:center;justify-content:center;height:90vh;text-align:center">
        <h3>Can't reach Flight Lab</h3><p style="opacity:.7">\(msg)</p>
        <button onclick="location.href='\(home.absoluteString)'" style="padding:12px 22px;font-size:16px;border-radius:10px;border:0">Try again</button></body>
        """, baseURL: nil)
    }

    func webView(_ w: WKWebView, runJavaScriptAlertPanelWithMessage m: String, initiatedByFrame f: WKFrameInfo, completionHandler: @escaping () -> Void) {
        present(UIAlertController(title: nil, message: m, preferredStyle: .alert), actions: [("OK", { completionHandler() })])
    }

    func webView(_ w: WKWebView, runJavaScriptConfirmPanelWithMessage m: String, initiatedByFrame f: WKFrameInfo, completionHandler: @escaping (Bool) -> Void) {
        present(UIAlertController(title: nil, message: m, preferredStyle: .alert),
                actions: [("Cancel", { completionHandler(false) }), ("OK", { completionHandler(true) })])
    }

    func webView(_ w: WKWebView, runJavaScriptTextInputPanelWithPrompt p: String, defaultText d: String?, initiatedByFrame f: WKFrameInfo, completionHandler: @escaping (String?) -> Void) {
        let a = UIAlertController(title: nil, message: p, preferredStyle: .alert)
        a.addTextField { $0.text = d }
        present(a, actions: [("Cancel", { completionHandler(nil) }), ("OK", { completionHandler(a.textFields?.first?.text) })])
    }

    private func present(_ a: UIAlertController, actions: [(String, () -> Void)]) {
        for (t, f) in actions { a.addAction(UIAlertAction(title: t, style: t == "Cancel" ? .cancel : .default) { _ in f() }) }
        var top = web?.window?.rootViewController
        while let p = top?.presentedViewController { top = p }
        top?.present(a, animated: true)
    }
}
