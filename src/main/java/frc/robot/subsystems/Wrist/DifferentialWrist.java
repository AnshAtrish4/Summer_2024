package frc.robot.subsystems.Wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DifferentialWrist {

    private CANSparkMax RightMotor;
    private CANSparkMax LeftMotor;

    private AbsoluteEncoder encoderL;
    private AbsoluteEncoder encoderR;
    private AbsoluteEncoder encoderMiddle;
    private AbsoluteEncoder encoderOuter;

    private ArmFeedforward feedforward;

    private SparkPIDController RPID;
    private SparkPIDController LPID;

    private double motorLOffset ;
    private double motorROffset ;

    public static DifferentialWrist instance;
    // State variables
    private double tiltPosition; // Current tilt angle (degrees)
    private double rotationPosition; // Current rotation angle (degrees)

    public DifferentialWrist() {

        motorLOffset = 0;
        motorROffset = 0;
        
        tiltPosition = 0.0;
        rotationPosition = 0.0;

        RightMotor = new CANSparkMax(1, MotorType.kBrushless);
        RightMotor.restoreFactoryDefaults();

        RightMotor.setSmartCurrentLimit(60);
        RightMotor.setIdleMode(IdleMode.kBrake);
        RightMotor.setInverted(false);

        LeftMotor = new CANSparkMax(2, MotorType.kBrushless);
        LeftMotor.restoreFactoryDefaults();

        LeftMotor.setSmartCurrentLimit(60);
        LeftMotor.setIdleMode(IdleMode.kBrake);
        LeftMotor.setInverted(true);

        encoderR = RightMotor.getAbsoluteEncoder();
        encoderL = LeftMotor.getAbsoluteEncoder();
        encoderMiddle = RightMotor.getAbsoluteEncoder();
        encoderOuter = LeftMotor.getAbsoluteEncoder();

        encoderR.setPositionConversionFactor(360);
        encoderL.setPositionConversionFactor(360);

        feedforward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

        RPID = RightMotor.getPIDController();

        RPID.setP(0.0);
        RPID.setI(0.0);
        RPID.setD(0.0);
        RPID.setFeedbackDevice(encoderR);
        RPID.setOutputRange(-1, 1);

        RightMotor.burnFlash();

        LPID = LeftMotor.getPIDController();

        LPID.setP(0.0);
        LPID.setI(0.0);
        LPID.setD(0.0);
        LPID.setFeedbackDevice(encoderL);
        LPID.setOutputRange(-1, 1);

        LeftMotor.burnFlash();

    }

    public void moveWristOne(double targetTilt, double targetRotation) {
        // Update target positions
        tiltPosition = targetTilt;
        rotationPosition = targetRotation;

        // Read current positions from absolute encoders
        double motorLCurrent = encoderL.getPosition();
        double motorRCurrent = encoderR.getPosition();

        // Calculate the current tilt and rotation
        double currentTilt = (motorLCurrent + motorRCurrent) / 2.0;
        double currentRotation = (motorRCurrent - motorLCurrent) / 2.0;

        // Calculate the necessary motor adjustments
        double motorLTarget = motorLCurrent + (targetTilt - currentTilt) - (targetRotation - currentRotation);
        double motorRTarget = motorRCurrent + (targetTilt - currentTilt) + (targetRotation - currentRotation);

        // Command the motors to move to the targets
        LPID.setReference(motorLTarget, CANSparkMax.ControlType.kPosition, 0,
                feedforward.calculate(encoderL.getPosition(), 0));
        RPID.setReference(motorRTarget, CANSparkMax.ControlType.kPosition, 0,
                feedforward.calculate(encoderR.getPosition(), 0));

        // Log data for debugging
        SmartDashboard.putNumber("Tilt Position (Target)", tiltPosition);
        SmartDashboard.putNumber("Rotation Position (Target)", rotationPosition);
        SmartDashboard.putNumber("Tilt Position (Current)", currentTilt);
        SmartDashboard.putNumber("Rotation Position (Current)", currentRotation);
        SmartDashboard.putNumber("Motor L Target", motorLTarget);
        SmartDashboard.putNumber("Motor R Target", motorRTarget);
    }

    public void moveWristTwo(double targetTilt, double targetRotation) {
        // Update target positions
        tiltPosition = targetTilt;
        rotationPosition = targetRotation;
        // Read current positions from absolute encoders
        double middleRotation = encoderMiddle.getPosition(); // Middle gear for rotation
        double outerGearPosition = encoderOuter.getPosition(); // Outer gear for combined tilt/rotation
        // Decouple tilt and rotation
        double currentTilt = outerGearPosition - middleRotation;  // Isolate tilt
        double currentRotation = middleRotation;  // Direct rotation reading
        // Calculate motor targets
        double motorLTarget = (targetTilt - currentTilt) + (targetRotation - currentRotation);
        double motorRTarget = (targetTilt - currentTilt) - (targetRotation - currentRotation);
        // Command motors to move to targets
        LPID.setReference(motorLTarget, CANSparkMax.ControlType.kPosition, 0,
                feedforward.calculate(encoderOuter.getPosition(), 0));
        RPID.setReference(motorRTarget, CANSparkMax.ControlType.kPosition, 0,
                feedforward.calculate(encoderOuter.getPosition(), 0));
        // Log for debugging
        SmartDashboard.putNumber("Target Tilt", targetTilt);
        SmartDashboard.putNumber("Target Rotation", targetRotation);
        SmartDashboard.putNumber("Current Tilt", currentTilt);
        SmartDashboard.putNumber("Current Rotation", currentRotation);
    }

    public void moveWristThree(double targetTilt, double targetRotation) {
        // Read current positions from encoders
        double readL = encoderL.getPosition(); // Left motor encoder
        double readR = encoderR.getPosition(); // Right motor encoder
        // Calculate the current pivot (average of the two encoders) and end effector (difference between them)
        double currentPivot = (readR + readR) / 2;
        double currentRotation = readL - readR;
        // Calculate motor targets based on desired tilt and rotation
        double motorLTarget = targetTilt - currentPivot + targetRotation - currentRotation;
        double motorRTarget = targetTilt - currentPivot - targetRotation + currentRotation;
        // Command motors to move to the target positions
        LPID.setReference(motorLTarget, CANSparkMax.ControlType.kPosition, 0,
                feedforward.calculate(encoderL.getPosition(), 0));
        RPID.setReference(motorRTarget, CANSparkMax.ControlType.kPosition, 0,
                feedforward.calculate(encoderR.getPosition(), 0));
        // Log for debugging
        SmartDashboard.putNumber("Target Tilt", targetTilt);
        SmartDashboard.putNumber("Target Rotation", targetRotation);
        SmartDashboard.putNumber("Current Tilt", currentPivot);
        SmartDashboard.putNumber("Current Rotation", currentRotation);
    }
    


    

    public void resetOffsets() {
        motorLOffset = encoderL.getPosition();
        motorROffset = encoderR.getPosition();
    }

    public void enableCompressor() {

    }

    public void disableCompressor() {

    }

    public void setWheelsOn() {
        RightMotor.set(.75);
        LeftMotor.set(.75);
    }

    public void setWheelsReverse() {
        RightMotor.set(-.25);
        LeftMotor.set(-.25);
    }

    public void setWheelsOff() {
        RightMotor.set(0.05);
        LeftMotor.set(0.05);
    }

    // Method to extend both cylinders
    public void extendCylinders() {

    }

    // Method to retract both cylinders
    public void retractCylinders() {

    }

    public void getSolLNumberF() {
    }

    public void getSolLNumberR() {
    }

    public void getSolRNumberF() {
    }

    public void getSolRNumberR() {
    }

    public static DifferentialWrist getInstance() {
        if (instance == null)
            instance = new DifferentialWrist();
        return instance;
    }
}
