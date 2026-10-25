import SwiftUI

/// Self-centring virtual stick. Reports x/y in -1...1 (y up = +1).
struct JoystickView: View {
    var label: String
    var size: CGFloat = 190
    var onChange: (CGPoint) -> Void

    @State private var knob: CGSize = .zero
    @State private var active = false

    private var radius: CGFloat { size / 2 }
    private var knobSize: CGFloat { size * 0.38 }

    var body: some View {
        ZStack {
            Circle()
                .fill(Color.white.opacity(0.06))
                .overlay(Circle().stroke(Color.white.opacity(active ? 0.45 : 0.2), lineWidth: 2))
            Path { p in
                p.move(to: CGPoint(x: radius, y: 14)); p.addLine(to: CGPoint(x: radius, y: size - 14))
                p.move(to: CGPoint(x: 14, y: radius)); p.addLine(to: CGPoint(x: size - 14, y: radius))
            }
            .stroke(Color.white.opacity(0.12), lineWidth: 1)
            Circle()
                .fill(active ? Color.accentColor : Color.white.opacity(0.75))
                .frame(width: knobSize, height: knobSize)
                .shadow(color: .black.opacity(0.4), radius: 6)
                .offset(knob)
            Text(label)
                .font(.caption2.weight(.semibold))
                .foregroundStyle(.white.opacity(0.45))
                .offset(y: radius + 14)
        }
        .frame(width: size, height: size)
        .contentShape(Circle())
        .gesture(
            DragGesture(minimumDistance: 0)
                .onChanged { g in
                    active = true
                    var dx = g.location.x - radius, dy = g.location.y - radius
                    let maxR = radius - knobSize / 2
                    let d = sqrt(dx * dx + dy * dy)
                    if d > maxR { dx *= maxR / d; dy *= maxR / d }
                    knob = CGSize(width: dx, height: dy)
                    onChange(CGPoint(x: dx / maxR, y: -dy / maxR))
                }
                .onEnded { _ in
                    active = false
                    withAnimation(.spring(response: 0.25, dampingFraction: 0.7)) { knob = .zero }
                    onChange(.zero)
                }
        )
    }
}
