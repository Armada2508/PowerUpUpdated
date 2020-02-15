package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
    public UsbCamera cam0;
    public UsbCamera cam1;

    public CameraSubsystem() {
        cam0 = CameraServer.getInstance().startAutomaticCapture(0);
        cam1 = CameraServer.getInstance().startAutomaticCapture(1);
        
        cam0.setResolution(256, 144);
        cam1.setResolution(256, 144);
        cam0.setFPS(8);
        cam1.setFPS(8);
        
    }

    @Override
    public void periodic() {
        
    }
}