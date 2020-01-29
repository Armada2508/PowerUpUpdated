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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.motion.*;
import frc.lib.vision.FOV;
import frc.lib.vision.Resolution;
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
    private NetworkTableEntry m_kP;
    private NetworkTableEntry m_kI;
    private NetworkTableEntry m_kD;


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
        initDashboard();
        m_driveSubsystem.configTalons();
        FollowTrajectory.config(Constants.kS, Constants.kV, Constants.kA, Constants.kP, Constants.kI, Constants.kD, Constants.kB, Constants.kZeta, Constants.kTrackWidth);
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
        m_kP = m_robotTab.add("kP", Constants.kP).getEntry();
        m_kI = m_robotTab.add("kI", Constants.kI).getEntry();
        m_kD = m_robotTab.add("kD", Constants.kD).getEntry();

        WPI_TalonSRX[] allTalons = m_driveSubsystem.getAllTalons();

        for (WPI_TalonSRX talon : allTalons) {
            talonEntries.add(m_sensorLoggerTab.add("Talon " + (talon.getDeviceID()),
                    talon.getMotorOutputVoltage())
                    .withWidget(BuiltInWidgets.kGraph)
                    .getEntry());
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

        if ((Timer.getFPGATimestamp() % Constants.kUpdateRate) / 0.02 < 1) {

            Random noise = new Random();
            m_gyroEntry.setDouble(m_driveSubsystem.getGyro().getFusedHeading() + (noise.nextDouble() / 10000));
            m_odometer.setDouble(m_driveSubsystem.getAverageDistance() + (noise.nextDouble() / 10000));

            int count = 0;
            for (NetworkTableEntry t : talonEntries) {
                t.setDouble(m_driveSubsystem.getAllTalons()[count].getMotorOutputVoltage() + (noise.nextDouble() / 10000));
                count++;
            }
        }
    }

    public void updateFromDashboard() {
        FollowTrajectory.configPID(m_kP.getDouble(Constants.kP), m_kI.getDouble(Constants.kI), m_kD.getDouble(Constants.kD));
    }


    public void startDashboardCapture() {
        Shuffleboard.startRecording();
    }

    public void stopDashboardCapture() {
        Shuffleboard.startRecording();
    }

    public void changeMode() {
        m_driveSubsystem.reset();
    }

    public Command getAutonomousCommand() {
/*
        FollowTrajectory followTrajectory = new FollowTrajectory();

        try {
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/Line.wpilib.json"));
            return followTrajectory.getCommand(m_driveSubsystem, trajectory, trajectory.getInitialPose());
        } catch (IOException e) {
            System.out.println(e);
            return new InstantCommand();
        }
    */
    
        return new FollowTarget(m_driveSubsystem,
            Constants.kTurn,
            Constants.kThrottle,
            Constants.kMaxFollowOutput,
            Constants.kTargetWidth,
            Constants.kTargetDistance,
            Constants.kLimelightFOV,
            Constants.kLimelighResolution);
    }

    public void printOdo() {
        System.out.println(m_driveSubsystem.getPose());
    }

    public void printPos() {
        System.out.println(m_driveSubsystem.getPositionLeft() + "\t" + m_driveSubsystem.getPositionRight());
    }


    public void printVel() {
        System.out.println(m_driveSubsystem.getWheelSpeeds());
    }
}
