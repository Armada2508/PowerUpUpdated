/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import com.ctre.phoenix.motion.TrajectoryPoint;

import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;

/**
 * An example command that uses an example subsystem.
 */
public class AutoTest extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_driveSubsystem;
  private Trajectory m_trajectory;
  private TrajectoryPoint[] m_trajectoryPoints;
  private BufferedTrajectoryPointStream m_bufferedStream = new BufferedTrajectoryPointStream();
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
    m_trajectory.transformBy(new Transform2d(new Translation2d(), new Rotation2d(m_trajectory.sample(0).poseMeters.getRotation().getRadians())));
    TrajectoryPoint trajectoryPoint = new TrajectoryPoint();
    m_bufferedStream.Clear();
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.kDriveBaseWidth/12.0);
    /*for (int i = 0; i <= m_trajectory.getTotalTimeSeconds()*1000 + Constants.kAutoTimeStepMs; i+= Constants.kAutoTimeStepMs) {
        System.out.println(i/Constants.kAutoTimeStepMs);
        trajectoryPoint.timeDur = Constants.kAutoTimeStepMs;
        Translation2d currentDriveBase = new Translation2d((Constants.kDriveBaseWidth/12.0)/2.0, 0.0).rotateBy((new Rotation2d(Math.toRadians(90+m_trajectory.sample(i/1000.0).poseMeters.getRotation().getDegrees()))));
        Translation2d futureDriveBase = new Translation2d((Constants.kDriveBaseWidth/12.0)/2.0, 0.0).rotateBy((new Rotation2d(Math.toRadians(90+m_trajectory.sample((i+Constants.kAutoTimeStepMs)/1000.0).poseMeters.getRotation().getDegrees()))));
        Translation2d currentPosition = m_trajectory.sample(i/1000.0).poseMeters.getTranslation();
        Translation2d futurePosition = m_trajectory.sample((i+Constants.kAutoTimeStepMs)/1000.0).poseMeters.getTranslation();
        double deltaLeft = currentPosition.plus(currentDriveBase).getDistance(futurePosition.plus(futureDriveBase));
        double deltaRight = currentPosition.minus(currentDriveBase).getDistance(futurePosition.minus(futureDriveBase));
        System.out.println(deltaLeft);
        System.out.println(deltaRight);
    }*/  
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
}
