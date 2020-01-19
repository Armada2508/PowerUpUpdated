package frc.lib.motion;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory {
      
    private DriveSubsystem m_driveSubsystem;
  
    private static SimpleMotorFeedforward m_feedforward;

    private static DifferentialDriveKinematics m_kinematics;
  
    private static DifferentialDriveVoltageConstraint voltageConstraint;
  
    private Trajectory m_trajectory;
  
    private static RamseteController m_controller;

    public static void config(double kS, double kV, double kA, double b, double zeta, double trackWidth, double maxMotorVoltage) {
        m_feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        m_kinematics = new DifferentialDriveKinematics(trackWidth);
        voltageConstraint = new DifferentialDriveVoltageConstraint(m_feedforward, m_kinematics, maxMotorVoltage);
        m_controller = new RamseteController(b, zeta);
    }
    
    public RamseteCommand getCommand(DriveSubsystem driveSubsystem, Trajectory trajectory) {
        m_driveSubsystem = driveSubsystem;
        m_trajectory = trajectory;
        return new RamseteCommand(
            trajectory,
            m_driveSubsystem::getPose,
            m_controller,
            m_feedforward,
            m_kinematics,
            m_driveSubsystem::getWheelSpeeds,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            m_driveSubsystem::setVoltage,
            m_driveSubsystem);
    }
}