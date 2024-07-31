package frc.robot.subsystems.launcher;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.LauncherConstants;
import frc.robot.Ports;

public class Launcher {

    public enum LauncherState {
        // AMP(-48.5, 1.0),
        AMP(-60.5, 1.0),
        ALTAMP(-55, 0.9),
        START(0, 0.0),
        TRAP(-70.04991149902344, 0.8),
        LONG(-13.25, 1.0),
        HANDOFF(9, 0.5),
        HOVER(-3, 1.0),
        TOSS(-22, .80),
        AUTOMIDSHOT(-12, 1.0),
        // height: ?
        AUTOLEFTSHOT(-13.5, 1.0),
        // height: 20.75
        AUTORIGHTSHOT(-13.5, 1.0),
        // height: ?7
        SPEAKER(-58, 1.0),
        ALTSPEAKER(-23, 1.0),
        INTERLOPE(0.0, 1.0),
        TEST(-13.25, 1.0);

        public double position;
        public double launchSpeed;

        private LauncherState(double position, double launchSpeed) {
            this.position = position;
            this.launchSpeed = launchSpeed;
        }
    }

    public enum LeBronTeam {
        CAVS(-0.25),
        LAKERS(-20);

        public double position;

        private LeBronTeam(double position) {
            this.position = position;
        }
    }

    double anglePower = 0.2;

    private CANSparkMax shootMotor1;
    private CANSparkMax shootMotor2;

    private CANSparkMax flicker;

    private CANSparkMax pivotMotor;

    private CANSparkMax lebronMotor;

    private double increment = 1.0;

    private ArmFeedforward feedForward;
    private ArmFeedforward lebronFeedForward;

    private SparkMaxPIDController pivotController1;

    private SparkMaxPIDController lebronController;

    private static RelativeEncoder encoder;

    private static RelativeEncoder boxScore;

    private boolean[] connections = new boolean[8];

    private static LauncherState launchState = LauncherState.START;
    private static LeBronTeam leBronTeam = LeBronTeam.CAVS;

    public static Launcher instance;

    private HashMap<Double, Double> lookupTable = new HashMap<>();
    private double[] bluePositions = new double[] { 1.62, 1.93, 2.34, 2.41, 2.63, 2.71, 2.94, 3.01, 3.3, 4.14 };

    

    public Launcher() {
        shootMotor1 = new CANSparkMax(Ports.shootMotor1, MotorType.kBrushless);
        shootMotor1.restoreFactoryDefaults();

        shootMotor1.setSmartCurrentLimit(60);
        shootMotor1.setIdleMode(IdleMode.kBrake);
        shootMotor1.setInverted(false);
        shootMotor1.burnFlash();

        shootMotor2 = new CANSparkMax(Ports.shootMotor2, MotorType.kBrushless);
        shootMotor2.restoreFactoryDefaults();

        shootMotor2.setSmartCurrentLimit(60);
        shootMotor2.setIdleMode(IdleMode.kBrake);
        shootMotor2.setInverted(false);
        shootMotor2.burnFlash();

        flicker = new CANSparkMax(Ports.flicker, MotorType.kBrushless);
        flicker.restoreFactoryDefaults();

        flicker.setSmartCurrentLimit(20);
        flicker.setIdleMode(IdleMode.kBrake);
        flicker.setInverted(false);
        flicker.burnFlash();

        pivotMotor = new CANSparkMax(Ports.pivotMotor, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();

        pivotMotor.setSmartCurrentLimit(60);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setInverted(true);
        pivotMotor.setOpenLoopRampRate(1);

        lebronMotor = new CANSparkMax(Ports.lebron, MotorType.kBrushless);
        lebronMotor.restoreFactoryDefaults();

        lebronMotor.setSmartCurrentLimit(20);
        lebronMotor.setIdleMode(IdleMode.kBrake);

        feedForward = new ArmFeedforward(0.012, 0.017, 0.0, 0.0);
        // u:.023 l:.011 mid:.017 ks:.012

        lebronFeedForward = new ArmFeedforward(0, 0, 0);

        encoder = pivotMotor.getEncoder();

        pivotController1 = pivotMotor.getPIDController();

        pivotController1.setP(LauncherConstants.pivotPCoefficient);
        pivotController1.setI(LauncherConstants.pivotICoefficient);
        pivotController1.setD(LauncherConstants.pivotDCoefficient);

        pivotController1.setFeedbackDevice(encoder);

        pivotController1.setOutputRange(-1, 1);

        lebronController = lebronMotor.getPIDController();

        boxScore = lebronMotor.getEncoder();
        boxScore.setPositionConversionFactor(1);

        lebronController.setFeedbackDevice(boxScore);

        lebronController.setOutputRange(-1, 1);

        lebronController.setP(LauncherConstants.lebronPCoefficient);
        lebronController.setI(LauncherConstants.lebronICoefficient);
        lebronController.setD(LauncherConstants.lebronDCoefficient);

        pivotMotor.burnFlash();
        lebronMotor.burnFlash();

        lookupTable.put(2.94, -10.25);
        lookupTable.put(2.41, -13.25);
        lookupTable.put(2.71, -8.25);
        lookupTable.put(1.62, -18.25);
        lookupTable.put(2.63, -8.25);
        lookupTable.put(2.34, -13.25);
        lookupTable.put(3.01, -10.25);
        lookupTable.put(3.3, -7.5);
        lookupTable.put(1.93, -16.25);
        lookupTable.put(4.14, -7.25);

    }

    public void updatePose() {
        double clampedPosition = MathUtil.clamp(launchState.position, MIN_POSITION, MAX_POSITION);
        pivotController1.setReference(clampedPosition, CANSparkMax.ControlType.kPosition, 0,
                feedForward.calculate(encoder.getPosition(), 0));
    }

    public void moveLeBron() {
        lebronController.setReference(leBronTeam.position, CANSparkMax.ControlType.kPosition, 0,
                lebronFeedForward.calculate(boxScore.getPosition(), 0));
    }

    public void interpolateAngle() {
        // Implement if necessary
    }

    public void eject() {
        shootMotor2.set(0);
        shootMotor1.set(launchState.launchSpeed);
    }

    public void setLeBronOn() {
        lebronMotor.set(-0.5);
    }

    public void setLeBronReverse() {
        lebronMotor.set(0.5);
    }

    public void setLeBronOff() {
        lebronMotor.set(0.0);
    }

    public void setPivotOff() {
        pivotMotor.set(0.0);
    }

    public double getTestPosition() {
        return LauncherState.TEST.position;
    }

    public double getLeBronPosition() {
        return boxScore.getPosition();
    }

    public void setLauncherOn() {
        if (launchState == LauncherState.AMP) {
            shootMotor1.set(launchState.launchSpeed * 0.1);
            shootMotor2.set(launchState.launchSpeed * 0.1);
        } else if (launchState == LauncherState.ALTAMP) {
            shootMotor1.set(-launchState.launchSpeed);
            shootMotor2.set(launchState.launchSpeed * 0.1);
        } else {
            shootMotor1.set(launchState.launchSpeed);
            shootMotor2.set(launchState.launchSpeed);
        }
    }

    public void setReverseLauncherOn() {
        shootMotor1.set(-launchState.launchSpeed);
        shootMotor2.set(-launchState.launchSpeed);
    }

    public void setLauncherOff() {
        shootMotor1.set(0.0);
        shootMotor2.set(0.0);
    }

    public void setFlickerOff() {
        flicker.set(0.0);
    }

    public void setFlickerOn() {
        flicker.set(1.0);
    }

    public void setFlickerReverse() {
        flicker.set(-1.0);
    }

    public void setState(LauncherState state) {
        launchState = state;
        if (launchState == LauncherState.TEST) {
            pivotMotor.set(-0.5);
        }
    }

    public void setLeBronTeam(LeBronTeam team) {
        leBronTeam = team;
        // Reset the LeBron angle to the specified position for the team
        lebronController.setReference(leBronTeam.position, CANSparkMax.ControlType.kPosition, 0,
                lebronFeedForward.calculate(boxScore.getPosition(), 0));
    }

    public static Launcher getInstance() {
        if (instance == null) {
            instance = new Launcher();
        }
        return instance;
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public void setPosition(double position) {
        encoder.setPosition(position);
    }

    public double getLeBronAngle() {
        return boxScore.getPosition();
    }

    public void setLeBronAngle(double angle) {
        boxScore.setPosition(angle);
    }

    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public void setLeBronSpeed(double speed) {
        lebronMotor.set(speed);
    }

    public void setIncrement(double value) {
        increment = value;
    }

    public double getIncrement() {
        return increment;
    }

   // Define constants for position limits
   private static final double MIN_POSITION = -60.0; // Minimum angle limit
    private static final double MAX_POSITION = 0.0;   // Maximum angle limit

public void adjustLauncherForDetectedTag(double distance) { 
    // Compute the position based on the detected distance
    double computedPosition = computePositionFromTag(distance);
    
    // Clamp the computed position to ensure it stays within valid limits
    double clampedPosition = MathUtil.clamp(computedPosition, MIN_POSITION, MAX_POSITION);

    // Set the launcher to the computed (and clamped) position
    pivotController1.setReference(clampedPosition, CANSparkMax.ControlType.kPosition, 0,
            feedForward.calculate(encoder.getPosition(), 0));
    }

private double computePositionFromTag(double distance) {
    // Define a base position and scaling factor
    double basePosition = -10.0; // Base position for a certain distance
    double scalingFactor = -2.0; // Amount to decrease per unit of distance

    // Compute the position based on the distance
    double computedPosition = basePosition - (scalingFactor * distance);
    
    // Optionally clamp the position to prevent exceeding limits
    computedPosition = Math.max(computedPosition, -60.0);

    return computedPosition;
}

}
