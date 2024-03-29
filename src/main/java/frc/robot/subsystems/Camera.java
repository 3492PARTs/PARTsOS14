package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
    private static Camera camInstance;

    public Camera() {
        //? Might cause memory leak on the rio, gonna have to look into it later. -R
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
