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
import java.util.logging.Logger;

public class DualCamera {
    private static final Logger logger = Logger.getLogger(DualCamera.class.getName());

    private PhotonCamera backCamera;
    private PhotonCamera frontCamera;

    private static final Transform2d frontCameraOffset = new Transform2d(new Translation2d(0.5, 0.2), new Rotation2d(0));
    private static final Transform2d backCameraOffset = new Transform2d(new Translation2d(-0.5, -0.2), new Rotation2d(Math.PI));

    private static DualCamera instance;

    private final Map<Integer, Pose2d> fiducialMap = new HashMap<>();

    private DualCamera() {
        backCamera = new PhotonCamera("BackCamera");
        frontCamera = new PhotonCamera("FrontCamera");

        // Initialize fiducial map
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

        Pose2d frontRobotPose = calculatePoseFromCameraResult(frontResult, frontCameraOffset);
        Pose2d backRobotPose = calculatePoseFromCameraResult(backResult, backCameraOffset);

        if (frontRobotPose != null && backRobotPose != null) {
            double avgX = (frontRobotPose.getX() + backRobotPose.getX()) / 2;
            double avgY = (frontRobotPose.getY() + backRobotPose.getY()) / 2;
            Rotation2d avgRotation = new Rotation2d(
                (frontRobotPose.getRotation().getRadians() + backRobotPose.getRotation().getRadians()) / 2
            );

            return new Pose2d(avgX, avgY, avgRotation);
        }

        return frontRobotPose != null ? frontRobotPose : backRobotPose != null ? backRobotPose : new Pose2d();
    }

    private Pose2d calculatePoseFromCameraResult(PhotonPipelineResult result, Transform2d cameraOffset) {
        if (result != null && result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Pose2d fiducialPose = fiducialMap.get(target.getFiducialId());

            if (fiducialPose != null) {
                Transform3d transform = target.getBestCameraToTarget().inverse();
                Pose2d cameraToTargetPose = new Pose2d(
                    transform.getTranslation().getX(),
                    transform.getTranslation().getY(),
                    transform.getRotation().toRotation2d()
                );

                Transform2d cameraToTargetTransform = new Transform2d(
                    new Translation2d(cameraToTargetPose.getX(), cameraToTargetPose.getY()),
                    cameraToTargetPose.getRotation()
                );

                return applyTransform(
                    applyTransform(fiducialPose, cameraToTargetTransform),
                    cameraOffset
                );
            }
        }
        return null;
    }

    private Pose2d applyTransform(Pose2d pose, Transform2d transform) {
        Translation2d transformTranslation = transform.getTranslation();
        Rotation2d transformRotation = transform.getRotation();

        Translation2d rotatedTranslation = transformTranslation.rotateBy(pose.getRotation());

        Translation2d newTranslation = pose.getTranslation().plus(rotatedTranslation);
        Rotation2d newRotation = pose.getRotation().plus(transformRotation);

        return new Pose2d(newTranslation, newRotation);
    }
}
