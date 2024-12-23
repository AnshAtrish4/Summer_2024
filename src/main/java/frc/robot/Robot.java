package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Vision.DualCamera;
// import frc.robot.subsystems.Vision.StreamCamera;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.swerve.Drivebase;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Elevator.elevator;
import frc.robot.subsystems.swerve.Drivebase.DriveState;
import org.littletonrobotics.junction.LoggedRobot;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.commands.PathPlannerAuto; 

public class Robot extends LoggedRobot {

  private Drivebase drivebase;
  private elevator Elevator;;
  private Launcher launcher;
  private Claw claw;


  private static XboxController driver;
  private static XboxController operator;

  private DualCamera dualCamera;
  // private StreamCamera intakeCamera;


  private Command m_autoSelected;

  

  private boolean useCurrentSpike;


  private SendableChooser<Command> m_chooser = new SendableChooser<>();



  @Override
  public void robotInit() {
    drivebase = Drivebase.getInstance();
    launcher = Launcher.getInstance();
    dualCamera = DualCamera.getInstance();
    // intakeCamera = StreamCamera.getInstance();
    Elevator = elevator.getInstance();
    claw = Claw.getInstance();

    driver = new XboxController(0);
    operator = new XboxController(1);
    
   //drivebase.resetOdometry(new Pose2d(1, 1, new Rotation2d(0)));



    SmartDashboard.putData("Auto choices", m_chooser);

    useCurrentSpike = false;
  }

  @Override

  public void robotPeriodic() {

    CommandScheduler.getInstance().run();

  //   if (dualCamera.isFrontConnected()) {
  //     SmartDashboard.putString("Front Camera", "Connected");
  //     PhotonPipelineResult frontResult = dualCamera.getFront();
  //     if (frontResult.hasTargets()) {
  //         SmartDashboard.putString("Front Camera Target", "Targets Spotted");
  //         PhotonTrackedTarget frontTarget = frontResult.getBestTarget();
  //         Transform3d frontTransform = frontTarget.getBestCameraToTarget();
  //         double frontDistance = frontTransform.getTranslation().getNorm();
  //         double frontYaw = frontTarget.getYaw();
  //         int frontFiducialId = frontTarget.getFiducialId();
  //         SmartDashboard.putNumber("Front Camera Distance", frontDistance);
  //         SmartDashboard.putNumber("Front Camera Yaw", frontYaw);
  //         SmartDashboard.putNumber("Front Camera Fiducial ID", frontFiducialId);
  //     } else if (frontResult.hasTargets() == false){
  //         SmartDashboard.putString("Front Camera Target", "No Targets");
  //     }
  // } else if(dualCamera.isFrontConnected() == false) {
  //     SmartDashboard.putString("Front Camera", "Not Connected");
  // }

  // if (dualCamera.isBackConnected()) {
  //     PhotonPipelineResult backResult = dualCamera.getBack();
  //     SmartDashboard.putString("Back Camera", "Connected");
  //     if (backResult.hasTargets()) {
  //         SmartDashboard.putString("Back Camera Target", "Targets Spotted");
  //         PhotonTrackedTarget backTarget = backResult.getBestTarget();
  //         Transform3d backTransform = backTarget.getBestCameraToTarget();
  //         double backDistance = backTransform.getTranslation().getNorm();
  //         double backYaw = backTarget.getYaw();
  //         int backFiducialId = backTarget.getFiducialId();
  //         SmartDashboard.putNumber("Back Camera Distance", backDistance);
  //         SmartDashboard.putNumber("Back Camera Yaw", backYaw);
  //         SmartDashboard.putNumber("Back Camera Fiducial ID", backFiducialId);
  //     } else if(backResult.hasTargets() == false) {
  //         SmartDashboard.putString("Back Camera Target", "No Targets");
  //     }
  // } else if(dualCamera.isBackConnected() == false) {
  //     SmartDashboard.putString("Back Camera", "Not Connected");
  // }

    SmartDashboard.putNumber("CRF", claw.cylinderR.getFwdChannel());
    SmartDashboard.putNumber("CRR", claw.cylinderR.getRevChannel());
    SmartDashboard.putNumber("CLF", claw.cylinderL.getFwdChannel());
    SmartDashboard.putNumber("CLR", claw.cylinderL.getRevChannel());
    SmartDashboard.putBoolean("Compressor enabled?", claw.isCompressorEnabled());

    // SmartDashboard.putNumber("Absolute Encoder", Elevator.getCumulativePosition() );
    // SmartDashboard.putNumber("Raw Encoder Position", Elevator.encoderA.getPosition());
    // SmartDashboard.putNumber("Cumulative Position", Elevator.getCumulativePosition());


            




    // if(intakeCamera.isIntakeCamConnected()){
    //   SmartDashboard.putBoolean("IntakeCamConnected", true);
      
    // }else{
    //   SmartDashboard.putBoolean("IntakeCamConnected", false);
    // }

    

    //  if (dualCamera.isBackConnected()) {
    //         PhotonPipelineResult backResult = dualCamera.getBack();
    //         if (backResult.hasTargets()) {
    //             PhotonTrackedTarget target = backResult.getBestTarget();
    //             SmartDashboard.putString("Back Camera Target", "Targets");
    //             Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    //             double distance  = bestCameraToTarget.getTranslation().getNorm();
    //             SmartDashboard.putNumber("Back to Target", distance);
    //             SmartDashboard.putNumber("Back Camera Target Yaw", target.getYaw());
    //             SmartDashboard.putNumber("Back Camera Target Pitch", target.getPitch());
    //             SmartDashboard.putNumber("Back Camera Target Area", target.getArea());
    //             SmartDashboard.putNumber("ID", target.getFiducialId());

    //         } else if (backResult.hasTargets() == false) {
    //             SmartDashboard.putString("Back Camera Target", "No Targets");
    //         }
    //     } else if(dualCamera.isBackConnected() == false){
    //         SmartDashboard.putString("Back Camera", "Not Connected");
    //     }

    //     if (dualCamera.isFrontConnected()) {
    //         PhotonPipelineResult frontResult = dualCamera.getFront();
    //         if (frontResult.hasTargets()) {
    //             PhotonTrackedTarget target = frontResult.getBestTarget();
    //             Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    //             double distance = bestCameraToTarget.getTranslation().getNorm();
    //             SmartDashboard.putString("Front Camera Target", "Targets");
    //             SmartDashboard.putNumber("Front to Target", distance);
    //             SmartDashboard.putNumber("Front Camera Target Yaw", target.getYaw());
    //             SmartDashboard.putNumber("Front Camera Target Pitch", target.getPitch());
    //             SmartDashboard.putNumber("Front Camera Target Area", target.getArea());
    //             SmartDashboard.putNumber("ID", target.getFiducialId());
    //         } else if (frontResult.hasTargets() == false){
    //             SmartDashboard.putString("Front Camera Target", "No Targets");
    //         }
    //     } else if(dualCamera.isFrontConnected() == false) {
    //         SmartDashboard.putString("Front Camera", "Not Connected");
    //     }

    Pose2d defaultPose = new Pose2d(0.0, 0.0, new Rotation2d(0));


        

    // Pose2d cameraPosition = dualCamera.calculateRobotPosition();
    // SmartDashboard.putNumber("Camera X Position", cameraPosition.getX());
    // SmartDashboard.putNumber("CameraX local", 14.5-cameraPosition.getX());
    // SmartDashboard.putNumber("Camera Y Position", cameraPosition.getY());
    // SmartDashboard.putNumber("Camera Heading", cameraPosition.getRotation().getDegrees());
    
    // if(cameraPosition!= defaultPose){
    //   drivebase.updateOdometry(cameraPosition);
    // }

    
    drivebase.periodic();

    if (isEnabled()) {
      claw.enableCompressor();  // Turn on the compressor when robot is enabled
  } else {
      claw.disableCompressor(); // Turn off the compressor when robot is disabled
  }


    SmartDashboard.putNumber("Gyro Angle:", (drivebase.getHeading() + 90) % 360);
    SmartDashboard.putNumber("X-coordinate", drivebase.getPose().getX());
    SmartDashboard.putNumber("Y-coordinate", drivebase.getPose().getY());

    SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());


  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();

    drivebase.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(m_chooser.getSelected().getName()));



    if (m_autoSelected != null) {
      m_autoSelected.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
 
  }

  @Override
  public void teleopInit() {
    if (m_autoSelected != null) {
      m_autoSelected.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
   

    /* DRIVE CONTROLS */

    double ySpeed = drivebase.inputDeadband(-driver.getLeftX());
    double xSpeed = drivebase.inputDeadband(driver.getLeftY());
    double rot = drivebase.inputDeadband(-driver.getRightX());

    if (driver.getAButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 180);
    } else if (driver.getBButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 270);
    } else if (driver.getYButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 0);
    } else if (driver.getXButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 90);
    } else if (driver.getLeftTriggerAxis() > 0) {
      drivebase.holdHeading(xSpeed, ySpeed);
    } else {
      drivebase.currHeading = -1;
      drivebase.drive(xSpeed, ySpeed, rot);
    }
  

    if (driver.getPOV() == 0) {
      drivebase.zeroHeading();
    }


    if (driver.getRightTriggerAxis() > 0) {
      drivebase.setDriveState(DriveState.SLOW);
    }

  //   if (operator.getPOV() == 270) {
  //     if (dualCamera.isFrontConnected()) {
  //         PhotonPipelineResult frontResult = dualCamera.getFront();
  //         if (frontResult.hasTargets()) {
  //             PhotonTrackedTarget frontTarget = frontResult.getBestTarget();
  //             Transform3d frontTransform = frontTarget.getBestCameraToTarget();
  //             double frontDistance = frontTransform.getTranslation().getNorm();
  //             int id = frontTarget.getFiducialId(); // Get the fiducial ID from the detected tag
              
  //             String alliance = DriverStation.getAlliance().toString();
              
  //             // Ensure alliance comparison is done using .equals for string comparison
  //             if (alliance.equals("RED")) {
  //                 if (id == 4) {
  //                     launcher.adjustLauncherForDetectedTag(frontDistance);
  //                 }
  //             } else if (alliance.equals("BLUE")) {
  //                 if (id == 7) {
  //                     launcher.adjustLauncherForDetectedTag(frontDistance);
  //                 }
  //             }
  //         }
  //     }
  // }

  if (driver.getRightBumper()){
    claw.extendCylinders();
  }else if (driver.getLeftBumper()){
    claw.retractCylinders();
  }
  
  }

  @Override
  public void disabledInit() {

    claw.disableCompressor();

  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

}