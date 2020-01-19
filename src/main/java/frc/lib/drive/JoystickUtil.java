package frc.lib.drive;

public class JoystickUtil {

    public static double deadband(double value, double threshold) {
        if(Math.abs(value) > threshold) {
            return value;
        } else {
            return 0.0;
        }
    }
}