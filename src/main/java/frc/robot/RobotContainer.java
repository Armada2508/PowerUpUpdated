/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.motion.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    ArrayList<NetworkTableEntry> talonEntries = new ArrayList();
    private Joystick m_joystick = new Joystick(Constants.kJoystickPort);
    private ShuffleboardTab m_robotTab = Shuffleboard.getTab("Robot");
    private ShuffleboardTab m_sensorLoggerTab = Shuffleboard.getTab("Logger");
    private NetworkTableEntry m_gyroEntry;
    private NetworkTableEntry m_odometer;


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

    public void initDashboard() {
        m_robotTab.add("kP", Constants.kP);
        m_robotTab.add("kI", Constants.kI);
        m_robotTab.add("kD", Constants.kD);
        m_robotTab.add("kF", Constants.kF);
        m_robotTab.add("kMaxIntegralAccumulator", Constants.kMIA);

        WPI_TalonSRX[] allTalons = m_driveSubsystem.getAllTalons();

        for (int i = 0; i < allTalons.length; i++) {
            WPI_TalonSRX talon = allTalons[i];
            talonEntries.add(m_sensorLoggerTab.add("Talon " + (talon.getDeviceID()), talon.getMotorOutputVoltage()).withWidget(BuiltInWidgets.kGraph).getEntry());
        }

        m_gyroEntry = m_sensorLoggerTab.add("Gyro", m_driveSubsystem.getGyro()
                .getFusedHeading())
                .withWidget(BuiltInWidgets.kGraph)
                .getEntry();

        m_odometer = m_sensorLoggerTab.add("Odometer", m_driveSubsystem.getAverageDistance())
                .withWidget(BuiltInWidgets.kGraph)
                .getEntry();
    }

    public void updateDashboard() {

        if (Timer.getFPGATimestamp() / 0.02 % (Constants.kUpdateRate / 0.02) < 1) {

            Random noise = new Random();
            m_gyroEntry.setDouble(m_driveSubsystem.getGyro().getFusedHeading() + (noise.nextDouble() / 10000));
            m_odometer.setDouble(m_driveSubsystem.getAverageDistance() + (noise.nextDouble() / 10000));

            int count = 0;
            for (NetworkTableEntry t : talonEntries) {
                t.setDouble(m_driveSubsystem.getAllTalons()[count].getMotorOutputVoltage());
                count++;
            }
        }
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
