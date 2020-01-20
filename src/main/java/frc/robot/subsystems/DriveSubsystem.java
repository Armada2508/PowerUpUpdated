/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motion.EncoderUtil;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */

  private final WPI_TalonSRX m_right = new WPI_TalonSRX(Constants.kRightMotorPort);
  private final WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(Constants.kRightMotorFollowerPort);
  private final WPI_TalonSRX m_left = new WPI_TalonSRX(Constants.kLeftMotorPort);
  private final WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(Constants.kLeftMotorFollowerPort);

  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_right, m_rightFollower);
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_left, m_leftFollower);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final PigeonIMU m_imu = new PigeonIMU(0);

  private final DifferentialDriveOdometry m_odometry;

  private ShuffleboardTab m_robotTab = Shuffleboard.getTab("Robot");

  private ComplexWidget m_driveWidget = m_robotTab.add("Drive", m_drive).withWidget(BuiltInWidgets.kDifferentialDrive);

  public DriveSubsystem() {
    m_right.setSensorPhase(Constants.kRightSensorInverted);
    m_left.setSensorPhase(Constants.kLeftSensorInverted);
    setInverted(Constants.kRightInverted, m_rightMotors);
    setInverted(Constants.kLeftInverted, m_leftMotors);

    m_drive.setSafetyEnabled(false);

    resetHeading();
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getPositionLeft(), getPositionRight());
  } 

  public void setPowers(double powerR, double powerL) {
    m_drive.tankDrive(powerL, powerR);
  }

  public void setVoltage(double voltsR, double voltsL) {
    System.out.println(voltsL + "\t" + voltsR);
    m_rightMotors.setVoltage(voltsR);
    m_leftMotors.setVoltage(voltsL);
  }
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getHeading() {
    return Math.IEEEremainder(m_imu.getFusedHeading(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getVelocityLeft(), getVelocityRight());
  }

  public void reset() {
    resetEncoders();
    resetTalons();
    resetHeading();
  }

  private void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void resetGyro() {
    m_imu.setFusedHeading(0);
  }

  public void resetEncoders() {
    resetEncoder(m_right);
    resetEncoder(m_left);
  }

  private void resetEncoder(TalonSRX talon) {
    talon.setSelectedSensorPosition(0);
  }

  private void resetHeading() {
    m_imu.setFusedHeading(0);
  }

  public void resetTalons() {
    m_right.configFactoryDefault();
    m_rightFollower.configFactoryDefault();
    m_left.configFactoryDefault();
    m_leftFollower.configFactoryDefault();
    m_right.set(ControlMode.PercentOutput, 0.0);
    m_left.set(ControlMode.PercentOutput, 0.0);
  }

  public void configTalons() {
    resetTalons();
    m_rightFollower.follow(m_right);
    m_leftFollower.follow(m_left);
    m_right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_right.configMotionProfileTrajectoryPeriod(0);
    m_left.configMotionProfileTrajectoryPeriod(0);
  }

  public double getPositionLeft() {
    return toDistance(m_left.getSelectedSensorPosition());
  }
  
  public double getPositionRight() {
    return toDistance(m_right.getSelectedSensorPosition());
  }

  public double getAverageDistance() {
    return (getPositionLeft() + getPositionRight()) / 2.0;
  }

  public double toDistance(int sensorPosition) {
    return EncoderUtil.toDistance(sensorPosition, Constants.kTicksPerRev, Constants.kGearRatio, Constants.kWheelDiameter);
  }

  public double getVelocityRight() {
   return toVelocity(m_right.getSelectedSensorVelocity());
  }
  
  public double getVelocityLeft() {
   return toVelocity(m_left.getSelectedSensorVelocity());
  }

  public double toVelocity(int velocity) {
    return EncoderUtil.toVelocity(velocity, Constants.kTicksPerRev, Constants.kGearRatio, Constants.kWheelDiameter, Constants.kSampleTime);
  }

  public void setConfigRight(double kP, double kI, double kD, double kF, double kMIA) {
    setConfig(kP, kI, kD, kF, kMIA, m_right);
  }
  
  public void setConfigLeft(double kP, double kI, double kD, double kF, double kMIA) {
    setConfig(kP, kI, kD, kF, kMIA, m_left);
  }
  
  public void setInvertedRight(boolean isInverted) {
    setInverted(isInverted, m_rightMotors);
  }

  public void setInvertedLeft(boolean isInverted) {
    setInverted(isInverted, m_leftMotors);
  }
  
  private void setConfig(double kP, double kI, double kD, double kF, double kMIA, TalonSRX talon) {
    talon.config_kP(Constants.kSlot, kP);
    talon.config_kI(Constants.kSlot, kI);
    talon.config_kD(Constants.kSlot, kD);
    talon.config_kF(Constants.kSlot, kF);
    talon.configMaxIntegralAccumulator(Constants.kSlot, kMIA);
  }

  private void setInverted(boolean isInverted, SpeedControllerGroup motors) {
    motors.setInverted(isInverted);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
}