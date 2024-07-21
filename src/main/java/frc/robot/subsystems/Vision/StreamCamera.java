package frc.robot.subsystems.Vision;

import java.util.stream.Stream;

import org.photonvision.PhotonCamera;

public class StreamCamera {
    private PhotonCamera intakeCam;
    private static StreamCamera instance;

    public StreamCamera(){
        intakeCam = new PhotonCamera("intakeCamera");
        intakeCam.setDriverMode(true);
    }

    public boolean isIntakeCamConnected(){
        return intakeCam.isConnected();
    }

    public static StreamCamera getInstance() {
        if (instance == null) {
          instance = new StreamCamera();
        }
        return instance;
      }
    
}
