package frc.robot.subsystems.Elevator;

import java.util.Set;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Ports;
import frc.robot.Constants;

public class elevator {

//     Tuning Process
// Set kS to Zero Initially:

// Start by setting kS to zero so you can isolate the effect of kG.
// Tune kG for Gravity Compensation:

// Gradually increase kG until the elevator begins to hold its position against gravity when stationary. This may take some trial and error.
// Keep adjusting until the elevator stops sliding downward and can hold in a steady position.
// Add kS to Overcome Static Friction:

// Once kG is set, incrementally increase kS to overcome any static friction or small inconsistencies in the hold.
// Keep kS low enough to prevent the elevator from moving upward but high enough to prevent any small downward drifts.
// Test at Different Heights:

// Test your tuning at various positions to ensure consistent holding across the elevator’s range of motion. Depending on your elevator’s weight distribution, you may need to make minor adjustments.

    public static elevator instance;
    private CANSparkMax elevatorMotorL;
    private CANSparkMax elevatorMotorR;
    public RelativeEncoder encoderL;
    public RelativeEncoder encoderR;
    private SparkMaxPIDController elevatorControllerL;
    private SparkMaxPIDController elevatorControllerR;
    private static ElevatorFeedforward feedForward;
    public ElevatorState elevatorState = ElevatorState.BOT;
    private boolean[] connections = new boolean[2];

    public AbsoluteEncoder encoderA;

    private double lastAbsolutePosition = 0.0; // Tracks the last read position
    private double cumulativePosition = 0.0;  // Tracks the cumulative position
    private static final double WRAP_THRESHOLD = 0.5; // Threshold to detect wraparound

    public DigitalInput minPosEncoder;

    public enum ElevatorState{
        //THESE ARE JUST GUESSES CHANGE THEM LATER
        GROUND(-5),
        BOT(-45),
        MID(-69),
        TOP(-86);

        public double position;
        private ElevatorState(double position){
            this.position = position;
        }
    }
    public elevator(){
        elevatorMotorL = new CANSparkMax(1, MotorType.kBrushless);
        elevatorMotorL.restoreFactoryDefaults();
        elevatorMotorL.setSmartCurrentLimit(60);
        elevatorMotorL.setIdleMode(IdleMode.kCoast);
        elevatorMotorL.setInverted(true);
        // elevatorMotorL.setOpenLoopRampRate(10);
        // elevatorMotorL.setClosedLoopRampRate(1);


        elevatorMotorR = new CANSparkMax(2, MotorType.kBrushless);
        elevatorMotorR.restoreFactoryDefaults();
        elevatorMotorR.setSmartCurrentLimit(60);
        elevatorMotorR.setIdleMode(IdleMode.kCoast);
        elevatorMotorR.setInverted(false);
        // elevatorMotorR.setOpenLoopRampRate(10);
        // elevatorMotorR.setClosedLoopRampRate(1);

        encoderA = elevatorMotorL.getAbsoluteEncoder(Type.kDutyCycle);
        encoderA.setPositionConversionFactor(100.0);
        encoderA.setInverted(false);
        //minPosEncoder  =  new DigitalInput(0);
        


        //JUST A GUESS TUNE LATER
        feedForward = new ElevatorFeedforward(-0.15, -0.18, 0.0, 0.0);

        //-.3 ks
        encoderL = elevatorMotorL.getEncoder();
        elevatorControllerL = elevatorMotorL.getPIDController();
        
        elevatorControllerL.setP(0);
        elevatorControllerL.setI(0);
        elevatorControllerL.setD(0);
        elevatorControllerL.setFeedbackDevice(encoderA);
        elevatorControllerL.setOutputRange(-.5, .5);
        encoderR = elevatorMotorR.getEncoder();
        elevatorControllerR = elevatorMotorR.getPIDController();
        
        elevatorControllerR.setP(0);
        elevatorControllerR.setI(0);
        elevatorControllerR.setD(0);
        elevatorControllerR.setFeedbackDevice(encoderA);
        elevatorControllerR.setOutputRange(-.5, .5);
        elevatorMotorL.burnFlash();
        elevatorMotorR.burnFlash();
    }

    

// In the updatePose method, use the cumulative position for the target position:
public void updatePose() {
    // Update the cumulative position based on the absolute encoder
    updateCumulativePosition();

    // Use the cumulative position for the target position
    double targetPosition = elevatorState.position;

    // Feedforward output to counteract gravity
    double feedForwardOutput = feedForward.calculate(targetPosition, 0);

    // Set the target position for both motors using cumulative position
    elevatorControllerL.setReference(cumulativePosition, CANSparkMax.ControlType.kPosition, 0, feedForwardOutput);
    elevatorControllerR.setReference(cumulativePosition, CANSparkMax.ControlType.kPosition, 0, feedForwardOutput);
}
    
    

    public double getCumulativePosition() {
        return cumulativePosition;
    }


    private void updateCumulativePosition() {
        double currentAbsolutePosition = encoderA.getPosition(); // Current raw encoder value
        double positionDelta = currentAbsolutePosition - lastAbsolutePosition;
    
        // Detect wraparound
        if (positionDelta > WRAP_THRESHOLD) {
            cumulativePosition -= 1.0; // Wrapped downward
        } else if (positionDelta < -WRAP_THRESHOLD) {
            cumulativePosition += 1.0; // Wrapped upward
        }
    
        // Add the delta to the cumulative position
        cumulativePosition += positionDelta;
    
        // Update the last absolute position
        lastAbsolutePosition = currentAbsolutePosition;
    }
    
    public void elevatorOn(){
        // // elevatorMotorL.set(-.25);
        // elevatorMotorR.set(-.25);
    }
    public void elevatorReverse(){
        // elevatorMotorL.set(.5);
        // elevatorMotorR.set(.5);
    }
    public void elevatorOff(){
        // elevatorMotorL.set(0);
        // elevatorMotorR.set(0);

        //  // Calculate the feedforward voltage to counteract gravity
        // double feedforwardVoltage = feedForward.calculate(0); // velocity = 0, since we're holding steady

        //  // Set the motors to the calculated voltage for holding
        // elevatorMotorL.setVoltage(feedforwardVoltage);
        // elevatorMotorR.setVoltage(feedforwardVoltage);
    }
    public void setElevatorPosition(double position){
        elevatorState.position = position;
    }
    public void setElevatorState(ElevatorState state){
        elevatorState = state;
    }
    public ElevatorState getElevatorState(){
        return elevatorState;
    }
    public double getPosition(RelativeEncoder encoder) {
        return encoder.getPosition();
    }
    public boolean hasReachedPose(double tolerance){
        return Math.abs(getPosition(encoderL) - elevatorState.position) < tolerance;
    }
    public boolean [] elevatorConnections(){
        if(elevatorMotorL.getBusVoltage() != 0){
            connections[0] = true;
        }else{
            connections[0] = false;
        }
        if(elevatorMotorR.getBusVoltage() != 0){
            connections[1] = true;
        }else{
            connections[1] = false;
        }
        return connections;
    }
    public static elevator getInstance() {
        if (instance == null)
            instance = new elevator();
        return instance;
    }
}