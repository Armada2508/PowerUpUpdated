package frc.lib.motion;

public class WheelPositions {
    private double m_RWheelPos;
    private double m_LWheelPos;

    
    public WheelPositions(double rWheelPos, double lWheelPos) {
        m_RWheelPos = rWheelPos;
        m_LWheelPos = lWheelPos;
    }

    public WheelPositions() {
        this(0.0, 0.0);
    }

    public double getRight() {
        return m_RWheelPos;
    }

    public double getLeft() {
        return m_LWheelPos;
    }
}