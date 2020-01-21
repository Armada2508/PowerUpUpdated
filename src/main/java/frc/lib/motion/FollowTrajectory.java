package frc.lib.motion;

import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.*;


/**
 * Keeps the robot on a fixed trajectory
 */
public class FollowTrajectory {

    private static SimpleMotorFeedforward m_feedforward;
    private static DifferentialDriveKinematics m_kinematics;
  
    private static RamseteController m_controller;
    private DriveSubsystem m_driveSubsystem;
    private Trajectory m_trajectory;

    /**
     * @param kS
     * @param kV
     * @param kA
     * @param b
     * @param zeta
     * @param trackWidth
     * @param maxMotorVoltage
     * Gets the current direction and movement of the robot and saves them to the static variables of the class
     */
    public static void config(double kS, double kV, double kA, double b, double zeta, double trackWidth, double maxMotorVoltage) {
        m_feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        m_kinematics = new DifferentialDriveKinematics(trackWidth);
        m_controller = new RamseteController(b, zeta);
    }

    /**
     *
     * @param driveSubsystem
     * @param trajectory
     * @return
     */
    public RamseteCommand getCommand(DriveSubsystem driveSubsystem, Trajectory trajectory) {
        m_driveSubsystem = driveSubsystem;
        m_trajectory = trajectory;
        return new RamseteCommand(
                trajectory,
                m_driveSubsystem::getPose,          // Equivalent Statement: () -> m_driveSubsystem.getPose(),
                m_controller,
                m_feedforward,
                m_kinematics,
                m_driveSubsystem::getWheelSpeeds,   // Equivalent Statement: () -> m_driveSubsystem.getWheelSpeeds(),
                new PIDController(Constants.kP, 0, Constants.kD),
                new PIDController(Constants.kP, 0, Constants.kD),
                m_driveSubsystem::setVoltage,       // Equivalent Statement: (voltsR, voltsL) -> m_driveSubsystem.setVoltage(voltsR, voltsL)
                m_driveSubsystem);
    }
}