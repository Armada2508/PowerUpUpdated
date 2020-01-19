/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.motion.FollowTrajectory;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private Joystick m_joystick = new Joystick(Constants.kJoystickPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_driveSubsystem.setDefaultCommand(new Drive(
      m_driveSubsystem,
      m_joystick,
      Constants.kThrottleAxis,
      Constants.kTurnAxis,
      Constants.kTurnBoostAxis,
      Constants.kThrottleInverted,
      Constants.kTurnInverted,
      Constants.kTurnBoostInverted,
      Constants.kMaxPower,
      Constants.kTurnRatio
    ));

  }

  private void init() {
    FollowTrajectory.config(Constants.kS, Constants.kV, Constants.kA, Constants.kB, Constants.kZeta, Constants.kTrackWidth, Constants.kMaxMotorVoltage);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("kP", Constants.kP);
    SmartDashboard.putNumber("kI", Constants.kI);
    SmartDashboard.putNumber("kD", Constants.kD);
    SmartDashboard.putNumber("kF", Constants.kF);
    SmartDashboard.putNumber("kMaxIntegralAccumulator", Constants.kMIA);
  }

  public void reset() {
    m_driveSubsystem.resetTalons();
  }

  public void printGyro() {
    System.out.println(m_driveSubsystem.getHeading());
  }

  public Command getAutonomousCommand() {

    FollowTrajectory followTrajectory = new FollowTrajectory();
    try {
      return followTrajectory.getCommand(m_driveSubsystem, TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/Path4.wpilib.json")));  
    } catch (IOException e) {
      return new InstantCommand();
    }
  }
}
