package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.HashMap;
import java.util.Map;

public class DualCamera {
    private PhotonCamera backCamera;
    private PhotonCamera frontCamera;

    private static final Transform2d frontCameraOffset = new Transform2d(new Translation2d(0.5, 0.2), new Rotation2d(0));
    private static final Transform2d backCameraOffset = new Transform2d(new Translation2d(-0.5, -0.2), new Rotation2d(Math.PI));

    private static DualCamera instance;

    private Map<Integer, Pose2d> fiducialMap = new HashMap<>();

    private DualCamera() {
        backCamera = new PhotonCamera("BackCamera");
        frontCamera = new PhotonCamera("FrontCamera");

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
    
        Pose2d frontRobotPose = null;
        Pose2d backRobotPose = null;
    
        if (frontResult != null && frontResult.hasTargets()) {
            PhotonTrackedTarget frontTarget = frontResult.getBestTarget();
            Pose2d frontFiducialPose = fiducialMap.get(frontTarget.getFiducialId());
    
            if (frontFiducialPose != null) {
                Transform3d frontTransform = frontTarget.getBestCameraToTarget().inverse();
                Pose2d frontCameraToTargetPose = new Pose2d(
                    frontTransform.getTranslation().getX(),
                    frontTransform.getTranslation().getY(),
                    frontTransform.getRotation().toRotation2d()
                );
    
                // Convert Pose2d to Transform2d if needed
                Transform2d frontCameraToTargetTransform = new Transform2d(
                    new Translation2d(frontCameraToTargetPose.getX(), frontCameraToTargetPose.getY()),
                    frontCameraToTargetPose.getRotation()
                );
    
                frontRobotPose = applyTransform(
                    applyTransform(frontFiducialPose, frontCameraToTargetTransform),
                    frontCameraOffset
                );
            }
        }
    
        if (backResult != null && backResult.hasTargets()) {
            PhotonTrackedTarget backTarget = backResult.getBestTarget();
            Pose2d backFiducialPose = fiducialMap.get(backTarget.getFiducialId());
    
            if (backFiducialPose != null) {
                Transform3d backTransform = backTarget.getBestCameraToTarget().inverse();
                Pose2d backCameraToTargetPose = new Pose2d(
                    backTransform.getTranslation().getX(),
                    backTransform.getTranslation().getY(),
                    backTransform.getRotation().toRotation2d()
                );
    
                // Convert Pose2d to Transform2d if needed
                Transform2d backCameraToTargetTransform = new Transform2d(
                    new Translation2d(backCameraToTargetPose.getX(), backCameraToTargetPose.getY()),
                    backCameraToTargetPose.getRotation()
                );
    
                backRobotPose = applyTransform(
                    applyTransform(backFiducialPose, backCameraToTargetTransform),
                    backCameraOffset
                );
            }
        }
    
        if (frontRobotPose != null && backRobotPose != null) {
            double avgX = (frontRobotPose.getX() + backRobotPose.getX()) / 2;
            double avgY = (frontRobotPose.getY() + backRobotPose.getY()) / 2;
            Rotation2d avgRotation = new Rotation2d(
                (frontRobotPose.getRotation().getRadians() + backRobotPose.getRotation().getRadians()) / 2
            );
    
            return new Pose2d(avgX, avgY, avgRotation);
        }
    
        if (frontRobotPose != null) {
            return frontRobotPose;
        }
        if (backRobotPose != null) {
            return backRobotPose;
        }
    
        return new Pose2d();
    }

    private Pose2d applyTransform(Pose2d pose, Transform2d transform) {
        // Extract translation and rotation from the transform
        Translation2d transformTranslation = transform.getTranslation();
        Rotation2d transformRotation = transform.getRotation();
    
        // Apply rotation to the translation
        Translation2d rotatedTranslation = transformTranslation.rotateBy(pose.getRotation());
    
        // Calculate new pose
        Translation2d newTranslation = pose.getTranslation().plus(rotatedTranslation);
        Rotation2d newRotation = pose.getRotation().plus(transformRotation);
    
        return new Pose2d(newTranslation, newRotation);
    }
    
}
