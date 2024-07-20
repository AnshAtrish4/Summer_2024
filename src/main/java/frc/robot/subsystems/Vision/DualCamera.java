package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.robot.subsystems.swerve.Drivebase;

public class DualCamera {
    private PhotonCamera backCamera;
    private PhotonCamera frontCamera;

    private static DualCamera instance;

    

    public DualCamera(){
        backCamera = new PhotonCamera("BackCamera");
        frontCamera = new PhotonCamera("FrontCamera");

    }

    public PhotonPipelineResult getBack(){
        return backCamera.getLatestResult();
    }

    public PhotonPipelineResult getFront(){
        return frontCamera.getLatestResult();
    }

    public boolean isBackConnected(){
        return backCamera.isConnected();
    }

    public boolean isFrontConnected(){
        return frontCamera.isConnected();
    }

      
    public Optional<Matrix<N3, N3>> getCameraMatrixForBack() {
        return backCamera.getCameraMatrix();
    }

   
    public Optional<Matrix<N3, N3>> getCameraMatrixForFront() {
        return frontCamera.getCameraMatrix();
    }

    public static DualCamera getInstance() {
    if (instance == null) {
      instance = new DualCamera();
    }
    return instance;
  }

    

    

    
}
