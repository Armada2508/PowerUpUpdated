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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  private ShuffleboardTab m_robotTab = Shuffleboard.getTab("Robot");

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

  public void robotInit() {
    m_driveSubsystem.configTalons();
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
    m_robotTab.add("kP", Constants.kP);
    m_robotTab.add("kI", Constants.kI);
    m_robotTab.add("kD", Constants.kD);
    m_robotTab.add("kF", Constants.kF);
    m_robotTab.add("kMaxIntegralAccumulator", Constants.kMIA);
  }

  public void startDashboardCapture() {
    Shuffleboard.startRecording();
  }

  public void stopDashboardCapture() {
    Shuffleboard.startRecording();
  }

  public void changeMode() {
    m_driveSubsystem.resetTalons();
    m_driveSubsystem.resetGyro();
    m_driveSubsystem.resetEncoders();
  }

  public Command getAutonomousCommand() {

    FollowTrajectory followTrajectory = new FollowTrajectory();
    
    try {
      return followTrajectory.getCommand(m_driveSubsystem, TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/Line.wpilib.json")));  
    } catch (IOException e) {
      System.out.println(e);
      return new InstantCommand();
    }
  }

  
  public void printPos() {
    System.out.println(m_driveSubsystem.getPositionLeft() + "\t" + m_driveSubsystem.getPositionRight());
  }

}
