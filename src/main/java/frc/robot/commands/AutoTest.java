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

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import java.util.*;

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
  /**
   * Creates a new Drive.
   *
   * @param subsystem The subsystem used by this command.
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
    m_trajectory.relativeTo(m_trajectory.sample(0.0).poseMeters);
    
    m_rightBufferedStream.Clear();
    m_leftBufferedStream.Clear();

    TrajectoryPoint rightTrajectoryPoint = new TrajectoryPoint();
    TrajectoryPoint leftTrajectoryPoint = new TrajectoryPoint();

    TrajectoryPointUtil.resetPoint(rightTrajectoryPoint);
    TrajectoryPointUtil.resetPoint(leftTrajectoryPoint);

    double leftDistance = 0.0;
    double rightDistance = 0.0;

    for (int i = 0; i <= m_trajectory.getTotalTimeSeconds()/(double)Constants.kAutoTimeStepMs; i++) {
      double time = i * Constants.kAutoTimeStepMs;
      rightTrajectoryPoint.zeroPos = ( i == 0 );
      leftTrajectoryPoint.zeroPos = ( i == 0 );
      rightTrajectoryPoint.isLastPoint = ( i > m_trajectory.getTotalTimeSeconds()/(double)Constants.kAutoTimeStepMs );
      leftTrajectoryPoint.isLastPoint = ( i > m_trajectory.getTotalTimeSeconds()/(double)Constants.kAutoTimeStepMs );
      System.out.println(i);
      WheelPositions wheelPositions = calcPos(time);
      rightDistance += wheelPositions.getRight();
      leftDistance += wheelPositions.getLeft();
      DifferentialDriveWheelSpeeds wheelSpeeds = calcSpeeds(time);
      rightTrajectoryPoint.timeDur = Constants.kAutoTimeStepMs;
      leftTrajectoryPoint.timeDur = Constants.kAutoTimeStepMs;
      rightTrajectoryPoint.position = rightDistance;
      leftTrajectoryPoint.position = leftDistance;
      rightTrajectoryPoint.velocity = wheelSpeeds.rightMetersPerSecond;
      leftTrajectoryPoint.velocity = wheelSpeeds.leftMetersPerSecond;
      
      System.out.println(rightTrajectoryPoint);

      /*m_rightBufferedStream.Write(rightTrajectoryPoint);
      m_leftBufferedStream.Write(leftTrajectoryPoint);*/

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    return;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }



  public WheelPositions calcPos(double time) {
    time /= 1000.0;
    Translation2d currentDriveBase = new Translation2d((Constants.kDriveBaseWidth/12.0)/2.0, 0.0).rotateBy((new Rotation2d(Math.toRadians(90+m_trajectory.sample(time).poseMeters.getRotation().getDegrees()))));
    Translation2d futureDriveBase = new Translation2d((Constants.kDriveBaseWidth/12.0)/2.0, 0.0).rotateBy((new Rotation2d(Math.toRadians(90+m_trajectory.sample(time+Constants.kAutoTimeStepMs).poseMeters.getRotation().getDegrees()))));
    Translation2d currentPosition = m_trajectory.sample(time).poseMeters.getTranslation();
    Translation2d futurePosition = m_trajectory.sample(time+Constants.kAutoTimeStepMs).poseMeters.getTranslation();
    double deltaLeft = currentPosition.plus(currentDriveBase).getDistance(futurePosition.plus(futureDriveBase));
    double deltaRight = currentPosition.minus(currentDriveBase).getDistance(futurePosition.minus(futureDriveBase));
    return new WheelPositions(deltaRight, deltaLeft);
  }

  public DifferentialDriveWheelSpeeds calcSpeeds(double time) {
    time /= 1000.0;
    ChassisSpeeds chassisSpeed = m_controller.calculate(m_trajectory.sample(time).poseMeters, m_trajectory.sample((time)+Constants.kAutoTimeStepMs));
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeed);
    return wheelSpeeds;
  }
}
