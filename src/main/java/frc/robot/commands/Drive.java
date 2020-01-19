/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.lib.drive.JoystickUtil;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class Drive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private Joystick m_joystick;
  private int m_throttleAxis;
  private int m_turnAxis;
  private int m_turnBoostAxis;
  private boolean m_throttleInverted;
  private boolean m_turnInverted;
  private boolean m_turnBoostInverted;
  private double m_maxPower;
  private double m_turnRatio;
  /**
   * Creates a new Drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(DriveSubsystem subsystem, Joystick joystick, int throttleAxis, int turnAxis, int turnBoostAxis, boolean throttleInverted, boolean turnInverted, boolean turnBoostInverted, double maxPower, double turnRatio) {
    m_driveSubsystem = subsystem;
    m_joystick = joystick;
    m_throttleAxis = throttleAxis;
    m_turnAxis = turnAxis;
    m_turnBoostAxis = turnBoostAxis;
    m_throttleInverted = throttleInverted;
    m_turnInverted = turnInverted;
    m_turnBoostInverted = turnBoostInverted;
    m_maxPower = maxPower;
    m_turnRatio = turnRatio;
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
    double throttle = m_joystick.getRawAxis(m_throttleAxis);
    double turn = m_joystick.getRawAxis(m_turnAxis);
    double turnBoost = m_joystick.getRawAxis(m_turnBoostAxis);
    
    System.out.println(throttle);

    throttle = JoystickUtil.deadband(throttle, Constants.kDeadbandThreshold);
    turn = JoystickUtil.deadband(turn, Constants.kDeadbandThreshold);
    turnBoost = JoystickUtil.deadband(turnBoost, Constants.kDeadbandThreshold);

    if(m_throttleInverted) throttle *= -1;
    if(m_turnInverted) turn *= -1;
    if(m_turnBoostInverted) turnBoost *= -1; 

    turn *= m_turnRatio;
    turnBoost *= m_turnRatio;
    turn += turnBoost; 

    double powerL = throttle + turn;
    double powerR = throttle - turn;

    powerR *= m_maxPower;
    powerL *= m_maxPower;

    double turningPower = powerL - powerR;
    if(turningPower > 0 && powerL > m_maxPower) {
      powerL = m_maxPower;
      powerR = m_maxPower - turningPower;
    } else if(turningPower < 0 && powerR > m_maxPower) {
      powerR = m_maxPower;
      powerL = m_maxPower + turningPower;
    }
    m_driveSubsystem.setPowers(powerR, powerL);
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
