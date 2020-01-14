/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Driving Constants
    public static final int kJoystickPort = 0;
    public static final int kThrottleAxis = 1;
    public static final int kTurnAxis = 2;
    public static final int kTurnBoostAxis = 0;
    public static final double kMaxPower = 0.75;
    public static final double kTurnRatio = 0.4;
    public static final boolean kThrottleInverted = true;
    public static final boolean kTurnInverted = false;
    public static final boolean kTurnBoostInverted = false;

    // Motion Profiling Constants
    public static final int kAutoTimeStepMs = 10;
    public static final double kDriveBaseWidth = 21.25;
    public static final double kB = 2.0;
    public static final double kZeta = 0.7;
}
