package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;

public class elevator {

    // Singleton instance
    public static elevator instance;

    private CANSparkMax elevatorMotorL;
    private CANSparkMax elevatorMotorR;

    public RelativeEncoder encoderL;
    public RelativeEncoder encoderR;

    private SparkMaxPIDController elevatorControllerL;
    private SparkMaxPIDController elevatorControllerR;

    private ElevatorFeedforward feedForward;

    public ElevatorState elevatorState = ElevatorState.BOT;

    // Trapezoidal profile variables
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State current;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;

    private double previousTime;

    public enum ElevatorState {
        GROUND(-5),
        BOT(-45),
        MID(-69),
        TOP(-86);

        public double position;

        private ElevatorState(double position) {
            this.position = position;
        }
    }

    public elevator() {
        // Initialize motors and encoders
        elevatorMotorL = new CANSparkMax(1, MotorType.kBrushless);
        elevatorMotorL.restoreFactoryDefaults();
        elevatorMotorL.setSmartCurrentLimit(60);
        elevatorMotorL.setIdleMode(CANSparkMax.IdleMode.kCoast);
        elevatorMotorL.setInverted(true);

        elevatorMotorR = new CANSparkMax(2, MotorType.kBrushless);
        elevatorMotorR.restoreFactoryDefaults();
        elevatorMotorR.setSmartCurrentLimit(60);
        elevatorMotorR.setIdleMode(CANSparkMax.IdleMode.kCoast);
        elevatorMotorR.setInverted(false);

        encoderL = elevatorMotorL.getEncoder();
        encoderR = elevatorMotorR.getEncoder();

        elevatorControllerL = elevatorMotorL.getPIDController();
        elevatorControllerL.setP(kP);
        elevatorControllerL.setI(kI);
        elevatorControllerL.setD(kD);
        elevatorControllerL.setOutputRange(-.5, .5);

        elevatorControllerR = elevatorMotorR.getPIDController();
        elevatorControllerR.setP(kP);
        elevatorControllerR.setI(kI);
        elevatorControllerR.setD(kD);
        elevatorControllerR.setOutputRange(-.5, .5);

        feedForward = new ElevatorFeedforward(-0.15, -0.18, 0.0, 0.0);

        // Set up trapezoidal constraints
        constraints = new TrapezoidProfile.Constraints(1.0, 0.5); // Max velocity and acceleration
        goal = new TrapezoidProfile.State(elevatorState.position, 0);
        current = new TrapezoidProfile.State(encoderL.getPosition(), 0);

        // Set initial time
        previousTime = System.nanoTime() / 1e9; // Time in seconds
    }

    public void updatePose() {
        // Update the profile goal based on elevatorState
        goal = new TrapezoidProfile.State(elevatorState.position, 0);
    
        // Calculate time elapsed
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - previousTime;
        previousTime = currentTime;
    
        // Create a new trapezoidal profile with the constraints, current state, and goal
        TrapezoidProfile profile = new TrapezoidProfile(constraints);
    
        // Calculate the next state in the trapezoidal profile using the time step
        TrapezoidProfile.State nextState = profile.calculate(dt, current, goal);
    
        // Calculate the feedforward value
        double feedforwardL = feedForward.calculate(nextState.position, nextState.velocity);
        double feedforwardR = feedForward.calculate(nextState.position, nextState.velocity);
    
        // Set the reference to the PID controllers for position control
        elevatorControllerL.setReference(nextState.position, CANSparkMax.ControlType.kPosition);
        elevatorControllerR.setReference(nextState.position, CANSparkMax.ControlType.kPosition);
    
        // Manually add the feedforward voltage to the motors
        elevatorMotorL.setVoltage(feedforwardL); 
        elevatorMotorR.setVoltage(feedforwardR);
    
        // Update the current state
        current = nextState;
    
        // Debugging information for SmartDashboard
        SmartDashboard.putNumber("Elevator Position", nextState.position);
        SmartDashboard.putNumber("Elevator Velocity", nextState.velocity);
    }
    
    



    public void setElevatorState(ElevatorState state) {
        elevatorState = state;
        current = new TrapezoidProfile.State(encoderL.getPosition(), 0);
        goal = new TrapezoidProfile.State(state.position, 0);
    }

    public ElevatorState getElevatorState() {
        return elevatorState;
    }

    public double getPosition(RelativeEncoder encoder) {
        return encoder.getPosition();
    }

    public boolean hasReachedPose(double tolerance) {
        return Math.abs(getPosition(encoderL) - elevatorState.position) < tolerance;
    }

    public static elevator getInstance() {
        if (instance == null)
            instance = new elevator();
        return instance;
    }
}
