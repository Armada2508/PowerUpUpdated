/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Motor Ports
    public static final int kRightMotorPort = 3;
    public static final int kRightMotorFollowerPort = 4;
    public static final int kLeftMotorPort = 7;
    public static final int kLeftMotorFollowerPort = 8;

    // Driving Constants
    public static final double kDeadbandThreshold = 0.06;
    public static final int kJoystickPort = 0;
    public static final int kThrottleAxis = 1;
    public static final int kTurnAxis = 2;
    public static final int kTurnBoostAxis = 0;
    public static final double kMaxPower = 1.0;
    public static final double kTurnRatio = 0.5;
    public static final boolean kThrottleInverted = true;
    public static final boolean kTurnInverted = false;
    public static final boolean kTurnBoostInverted = false;

    // Drive System Constants
    public static final boolean kRightInverted = true;
    public static final boolean kLeftInverted = false;
    public static final boolean kRightSensorInverted = false;
    public static final boolean kLeftSensorInverted = true;

    // Trajectory Following Constants
    public static final double kS = 0.93;
    public static final double kV = 1.12;
    public static final double kA = 0.118;
    public static final double kTrackWidth = 21.25/12.0;
    public static final double kB = 2;  
    public static final double kZeta = 0.7;
//    public static final double kTicksPerFt = 2882.07823;
    public static final int kTicksPerRev = 4096;
    public static final double kWheelDiameter = 0.5;
    public static final double kGearRatio = 42.0/38.0;
    public static final double kSampleTime = 0.1;
    public static final double kMaxMotorVoltage = 0.0;

    // PID
    public static final int kSlot = 0;
    public static final double kF = 0.0;
    public static final double kP = 0.00673;
    public static final double kI = 0.0;
    public static final double kD = 0.00167;
    public static final int kMIA = 200;

    // Gyro Settings
    public static final boolean kGyroReversed = false;


}
