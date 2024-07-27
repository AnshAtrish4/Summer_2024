package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.HashMap;
import java.util.Map;

public class DualCamera {
    private PhotonCamera backCamera;
    private PhotonCamera frontCamera;

    private static DualCamera instance;

    // Initialize the fiducial map using HashMap
    private Map<Integer, Pose2d> fiducialMap = new HashMap<>();

    private DualCamera() {
        backCamera = new PhotonCamera("BackCamera");
        frontCamera = new PhotonCamera("FrontCamera");

        // Populate the fiducial map
        fiducialMap.put(1, new Pose2d(593.68 / 1000, 9.68 / 1000, new Rotation2d(Math.toRadians(120))));
        fiducialMap.put(2, new Pose2d(637.21 / 1000, 34.79 / 1000, new Rotation2d(Math.toRadians(120))));
        fiducialMap.put(3, new Pose2d(652.73 / 1000, 196.17 / 1000, new Rotation2d(Math.toRadians(180))));
        fiducialMap.put(4, new Pose2d(652.73 / 1000, 218.42 / 1000, new Rotation2d(Math.toRadians(180))));
        fiducialMap.put(5, new Pose2d(578.77 / 1000, 323.00 / 1000, new Rotation2d(Math.toRadians(270))));
        fiducialMap.put(6, new Pose2d(72.50 / 1000, 323.00 / 1000, new Rotation2d(Math.toRadians(270))));
        fiducialMap.put(7, new Pose2d(-1.50 / 1000, 218.42 / 1000, new Rotation2d(Math.toRadians(0))));
        fiducialMap.put(8, new Pose2d(-1.50 / 1000, 196.17 / 1000, new Rotation2d(Math.toRadians(0))));
        fiducialMap.put(9, new Pose2d(14.02 / 1000, 34.79 / 1000, new Rotation2d(Math.toRadians(60))));
        fiducialMap.put(10, new Pose2d(57.54 / 1000, 9.68 / 1000, new Rotation2d(Math.toRadians(60))));
        fiducialMap.put(11, new Pose2d(468.69 / 1000, 146.19 / 1000, new Rotation2d(Math.toRadians(300))));
        fiducialMap.put(12, new Pose2d(468.69 / 1000, 177.10 / 1000, new Rotation2d(Math.toRadians(60))));
        fiducialMap.put(13, new Pose2d(441.74 / 1000, 161.62 / 1000, new Rotation2d(Math.toRadians(180))));
        fiducialMap.put(14, new Pose2d(209.48 / 1000, 161.62 / 1000, new Rotation2d(Math.toRadians(0))));
        fiducialMap.put(15, new Pose2d(182.73 / 1000, 177.10 / 1000, new Rotation2d(Math.toRadians(120))));
        fiducialMap.put(16, new Pose2d(182.73 / 1000, 146.19 / 1000, new Rotation2d(Math.toRadians(240))));
    }

    public static DualCamera getInstance() {
        if (instance == null) {
            instance = new DualCamera();
        }
        return instance;
    }

    public PhotonPipelineResult getBack() {
        return backCamera.getLatestResult();
    }

    public PhotonPipelineResult getFront() {
        return frontCamera.getLatestResult();
    }

    public boolean isBackConnected() {
        return backCamera.isConnected();
    }

    public boolean isFrontConnected() {
        return frontCamera.isConnected();
    }

    public Pose2d calculateRobotPosition() {
        PhotonPipelineResult frontResult = getFront();
        PhotonPipelineResult backResult = getBack();

        if (frontResult != null && frontResult.hasTargets() && backResult != null && backResult.hasTargets()) {
            PhotonTrackedTarget frontTarget = frontResult.getBestTarget();
            PhotonTrackedTarget backTarget = backResult.getBestTarget();

            Pose2d frontFiducialPose = fiducialMap.get(frontTarget.getFiducialId());
            Pose2d backFiducialPose = fiducialMap.get(backTarget.getFiducialId());

            if (frontFiducialPose == null || backFiducialPose == null) {
                return new Pose2d(); // Default position if fiducial ID not found
            }

            Transform3d frontTransform = frontTarget.getBestCameraToTarget().inverse();
            Pose2d frontRobotPose = frontFiducialPose.transformBy(new Transform2d(
                frontTransform.getTranslation().toTranslation2d(),
                frontTransform.getRotation().toRotation2d()
            ));

            Transform3d backTransform = backTarget.getBestCameraToTarget().inverse();
            Pose2d backRobotPose = backFiducialPose.transformBy(new Transform2d(
                backTransform.getTranslation().toTranslation2d(),
                backTransform.getRotation().toRotation2d()
            ));

            // Average the positions and rotations from both cameras
            double avgX = (frontRobotPose.getX() + backRobotPose.getX()) / 2;
            double avgY = (frontRobotPose.getY() + backRobotPose.getY()) / 2;
            Rotation2d avgRotation = new Rotation2d(
                (frontRobotPose.getRotation().getRadians() + backRobotPose.getRotation().getRadians()) / 2
            );

            return new Pose2d(avgX, avgY, avgRotation);
        }

        return new Pose2d(); // Default position if no targets are found
    }
}
