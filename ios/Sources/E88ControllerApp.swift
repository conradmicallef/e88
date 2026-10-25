import SwiftUI

@main
struct E88ControllerApp: App {
    @StateObject private var link = DroneLink()
    @StateObject private var settings = AppSettings()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(link)
                .environmentObject(settings)
                .persistentSystemOverlays(.hidden)
        }
    }
}
