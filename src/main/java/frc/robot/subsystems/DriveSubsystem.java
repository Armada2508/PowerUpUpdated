/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public TalonSRX left = new TalonSRX(7);
  private TalonSRX leftFollower = new TalonSRX(8);
  public TalonSRX right = new TalonSRX(3);
  private TalonSRX rightFollower = new TalonSRX(4);

  public DriveSubsystem() {
    left.configFactoryDefault();
    leftFollower.configFactoryDefault();
    right.configFactoryDefault();
    rightFollower.configFactoryDefault();
    leftFollower.follow(left);
    rightFollower.follow(right);
    left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    left.configMotionProfileTrajectoryPeriod(0);
    right.configMotionProfileTrajectoryPeriod(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double powerL, double powerR) {
    left.set(ControlMode.PercentOutput, powerL);
    right.set(ControlMode.PercentOutput, powerR);
  }

  public void reset() {
    left.configFactoryDefault();
    right.configFactoryDefault();
    left.clearMotionProfileTrajectories();
    right.clearMotionProfileTrajectories();
    leftFollower.clearMotionProfileTrajectories();
    rightFollower.clearMotionProfileTrajectories();
    right.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
    left.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
  }
}
