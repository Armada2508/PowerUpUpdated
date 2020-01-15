/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.lib.motion.*;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;

/**
 * An example command that uses an example subsystem.
 */
public class AutoTest extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_driveSubsystem;
  private Trajectory m_trajectory;
  private BufferedTrajectoryPointStream m_rightBufferedStream = new BufferedTrajectoryPointStream();
  private BufferedTrajectoryPointStream m_leftBufferedStream = new BufferedTrajectoryPointStream();
  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.kDriveBaseWidth/12.0);
  private RamseteController m_controller = new RamseteController(Constants.kB, Constants.kZeta);
  private MotionProfileStatus m_StatusR = new MotionProfileStatus();
  private MotionProfileStatus m_StatusL = new MotionProfileStatus();

  /**
   * Creates a new motion profile.
   *
   * @param subsystem The subsystem used by this command.
   * 
   * @param trajectory The trajectory to follow.
   * 
   */
  public AutoTest(DriveSubsystem subsystem, Trajectory trajectory) {
    m_driveSubsystem = subsystem;
    m_trajectory = trajectory;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    m_rightBufferedStream.Clear();
    m_leftBufferedStream.Clear();

    TrajectoryPoint rightTrajectoryPoint = new TrajectoryPoint();
    TrajectoryPoint leftTrajectoryPoint = new TrajectoryPoint();

    TrajectoryPointUtil.resetPoint(rightTrajectoryPoint);
    TrajectoryPointUtil.resetPoint(leftTrajectoryPoint);

    double leftDistance = 0.0;
    double rightDistance = 0.0;

    System.out.println("Generating Motion Profile");

    for (int i = 0; i <= m_trajectory.getTotalTimeSeconds()/(Constants.kAutoTimeStepMs/1000.0); i++) {
      // Calculate the time in milliseconds
      double time = i * Constants.kAutoTimeStepMs;
      rightTrajectoryPoint.zeroPos = ( i == 0 );
      leftTrajectoryPoint.zeroPos = ( i == 0 );
      rightTrajectoryPoint.isLastPoint = ( i > m_trajectory.getTotalTimeSeconds()/(Constants.kAutoTimeStepMs/1000.0));
      leftTrajectoryPoint.isLastPoint = ( i > m_trajectory.getTotalTimeSeconds()/(Constants.kAutoTimeStepMs/1000.0));
      WheelPositions wheelPositions = calcPos(time);
      rightDistance += wheelPositions.getRight();
      leftDistance += wheelPositions.getLeft();
      DifferentialDriveWheelSpeeds wheelSpeeds = calcSpeeds(time);
      rightTrajectoryPoint.timeDur = Constants.kAutoTimeStepMs;
      leftTrajectoryPoint.timeDur = Constants.kAutoTimeStepMs;
      rightTrajectoryPoint.position = rightDistance*Constants.kTicksPerFt;
      leftTrajectoryPoint.position = leftDistance*Constants.kTicksPerFt;
      rightTrajectoryPoint.velocity = wheelSpeeds.rightMetersPerSecond*Constants.kTicksPerFt/10;
      leftTrajectoryPoint.velocity = wheelSpeeds.leftMetersPerSecond*Constants.kTicksPerFt/10;


      m_rightBufferedStream.Write(rightTrajectoryPoint);
      m_leftBufferedStream.Write(leftTrajectoryPoint);

    }
    
    System.out.println("Done Generating Motion Profile");

    m_driveSubsystem.right.config_kP(Constants.kSlot, Constants.kP);
    m_driveSubsystem.right.config_kI(Constants.kSlot, Constants.kI);
    m_driveSubsystem.right.config_kD(Constants.kSlot, Constants.kD);
    m_driveSubsystem.right.config_kF(Constants.kSlot, Constants.kF);
    m_driveSubsystem.right.configMaxIntegralAccumulator(Constants.kSlot, Constants.kIntegralAccumulatorLimit);
    
    m_driveSubsystem.left.config_kP(Constants.kSlot, Constants.kP);
    m_driveSubsystem.left.config_kI(Constants.kSlot, Constants.kI);
    m_driveSubsystem.left.config_kD(Constants.kSlot, Constants.kD);
    m_driveSubsystem.left.config_kF(Constants.kSlot, Constants.kF);
    m_driveSubsystem.left.configMaxIntegralAccumulator(Constants.kSlot, Constants.kIntegralAccumulatorLimit);

    m_driveSubsystem.right.setSensorPhase(false);
    m_driveSubsystem.left.setSensorPhase(true);
    
    m_driveSubsystem.right.startMotionProfile(m_rightBufferedStream, Constants.kMinBufferedPoints, ControlMode.MotionProfile);
    m_driveSubsystem.left.startMotionProfile(m_leftBufferedStream, Constants.kMinBufferedPoints, ControlMode.MotionProfile);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.right.getMotionProfileStatus(m_StatusR);
    m_driveSubsystem.left.getMotionProfileStatus(m_StatusL);
    System.out.println(m_StatusR.isLast);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    return;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_StatusR.isLast && m_StatusL.isLast) {
      System.out.println("Done!");
    }
    return (m_StatusR.isLast && m_StatusL.isLast);
  }



  public WheelPositions calcPos(double time) {
    time /= 1000.0;
    
    Translation2d currentDriveBase;
    Translation2d futureDriveBase;

    if(time > 0) {
      currentDriveBase = new Translation2d((Constants.kDriveBaseWidth/12.0)/2.0, 0.0).rotateBy(new Rotation2d(Math.toRadians(90)+m_trajectory.sample(time).poseMeters.getRotation().getRadians()));
    } else {
      currentDriveBase = new Translation2d((Constants.kDriveBaseWidth/12.0)/2.0, 0.0).rotateBy(new Rotation2d(Math.toRadians(90)+m_trajectory.getInitialPose().getRotation().getRadians()));
    }

    if(time + Constants.kAutoTimeStepMs > 0) {
      futureDriveBase = new Translation2d((Constants.kDriveBaseWidth/12.0)/2.0, 0.0).rotateBy(new Rotation2d(Math.toRadians(90)+m_trajectory.sample(time+(Constants.kAutoTimeStepMs/1000.0)).poseMeters.getRotation().getRadians()));
    } else {
      futureDriveBase = new Translation2d((Constants.kDriveBaseWidth/12.0)/2.0, 0.0).rotateBy(new Rotation2d(Math.toRadians(90)+m_trajectory.getInitialPose().getRotation().getRadians()));
    }

    Translation2d currentPosition;
    Translation2d futurePosition;

    if (time > 0) {
      currentPosition = m_trajectory.sample(time).poseMeters.getTranslation();
    } else {
      currentPosition = m_trajectory.getInitialPose().getTranslation();
    }
    if (time + Constants.kAutoTimeStepMs > 0) {
      futurePosition = m_trajectory.sample(time+(Constants.kAutoTimeStepMs/1000.0)).poseMeters.getTranslation();
    } else {
      futurePosition = m_trajectory.getInitialPose().getTranslation();
    }

    double deltaLeft = currentPosition.plus(currentDriveBase).getDistance(futurePosition.plus(futureDriveBase));
    double deltaRight = currentPosition.minus(currentDriveBase).getDistance(futurePosition.minus(futureDriveBase));
    return new WheelPositions(deltaRight, deltaLeft);
  }

  public DifferentialDriveWheelSpeeds calcSpeeds(double time) {
    time /= 1000.0;
    ChassisSpeeds chassisSpeed = m_controller.calculate(m_trajectory.sample(time).poseMeters, m_trajectory.sample(time+(Constants.kAutoTimeStepMs/1000.0)));
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeed);
    return wheelSpeeds;
  }
}
