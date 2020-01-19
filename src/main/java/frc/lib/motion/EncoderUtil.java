package frc.lib.motion;

public class EncoderUtil {

    public static double toDistance(int sensorPosition, int encoderUnitsPerRev, double gearRatio, double wheelDiameter) {
        return (sensorPosition/encoderUnitsPerRev)*gearRatio*Math.PI*wheelDiameter;
    }
    public static double toVelocity(double velocity, int encoderUnitsPerRev, double gearRatio, double wheelDiameter, double time) {
        return (velocity/encoderUnitsPerRev)*gearRatio*Math.PI*wheelDiameter;
    }

}