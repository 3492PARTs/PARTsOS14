package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
    static Camera camInstance;

    public Camera() {
        //NetworkTableInstance.getDefault().getEntry("/CameraPublisher/FishEyes/streams").setStringArray(new String[] { "mjpg:http://roborio-3492-frc.local:1181/?action=stream" });
        CameraServer.startAutomaticCapture();
    }

    public static Camera getInstance() {
        if (camInstance == null) {
            camInstance = new Camera();
        }
        return camInstance;
    }
}
