/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.lib.vision.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowTarget extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final double m_kTurn;
  private final double m_kThrottle;
  private final double m_maxOutput;
  private final double m_targetWidth;
  private final double m_targetDistance;
  private final NetworkTableInstance m_instance;
  private final NetworkTable m_limelight;
  private FOV m_fov;
  private Resolution m_res;

  /**
   * Creates a new FollowTarget.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FollowTarget(DriveSubsystem subsystem, double kTurn, double kThrottle, double maxOutput, double targetWidth, double targetDistance, FOV fov, Resolution res) {
    m_driveSubsystem = subsystem;
    m_kTurn = kTurn;
    m_kThrottle = kThrottle;
    m_maxOutput = maxOutput;
    m_targetWidth = targetWidth;
    m_targetDistance = targetDistance;
    m_fov = fov;
    m_res = res;

    m_instance = NetworkTableInstance.getDefault();
    m_limelight = m_instance.getTable("limelight");

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableEntry targetFound = m_limelight.getEntry("tv");
    if(targetFound.getDouble(0) == 1) {
      NetworkTableEntry tx = m_limelight.getEntry("tx");
      NetworkTableEntry tw = m_limelight.getEntry("thor");
      double angleLeft = VisionUtil.pixelsToAngles(VisionUtil.anglesToPixels(tx.getDouble(0.0), m_fov.getX(), m_res.getX())-tw.getDouble(0.0)/2.0, m_fov.getX(), m_res.getX());
      double angleRight = VisionUtil.pixelsToAngles(VisionUtil.anglesToPixels(tx.getDouble(0.0), m_fov.getX(), m_res.getX())+tw.getDouble(0.0)/2.0, m_fov.getX(), m_res.getX());
      double widthAngle = angleRight-angleLeft;
      double distance = (m_targetWidth / 2.0) / (Math.tan(Math.toRadians((widthAngle / 2.0))));
      //System.out.println(distance + "; " + angleLeft + "..." + angleRight + "; " + widthAngle);
      
      double throttle = (distance - m_targetDistance) * m_kThrottle;
      double turn = tx.getDouble(0) * m_kTurn;
      if(Math.abs(throttle) > m_maxOutput) {
        throttle = m_maxOutput * Math.signum(throttle);
      }
      if(Math.abs(turn) > m_maxOutput) {
        turn = m_maxOutput * Math.signum(turn);
      }
      m_driveSubsystem.setArcade(throttle, turn);
    } else {
      m_driveSubsystem.setArcade(0, 0);
    }
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
