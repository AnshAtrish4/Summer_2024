package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ElevatorConstants;

public class elevator {

    public static elevator instance;
    private CANSparkMax elevatorMotorL;
    private CANSparkMax elevatorMotorR;
    public RelativeEncoder encoderL;
    public RelativeEncoder encoderR;
    private static ProfiledPIDController profiledPIDControllerL;
    private static ProfiledPIDController profiledPIDControllerR;
    private static ElevatorFeedforward feedForward;
    public ElevatorState elevatorState = ElevatorState.BOT;

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
        elevatorMotorL = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
        elevatorMotorL.restoreFactoryDefaults();
        elevatorMotorR = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);
        elevatorMotorR.restoreFactoryDefaults();

        encoderL = elevatorMotorL.getEncoder();
        encoderR = elevatorMotorR.getEncoder();

        // Initialize the ProfiledPIDControllers
        TrapezoidProfile.Constraints constraints = 
            new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);

        profiledPIDControllerL = new ProfiledPIDController(
            ElevatorConstants.elevatorPCoefficient,
            ElevatorConstants.elevatorICoefficient,
            ElevatorConstants.elevatorDCoefficient,
            constraints
        );

        profiledPIDControllerR = new ProfiledPIDController(
            ElevatorConstants.elevatorPCoefficient,
            ElevatorConstants.elevatorICoefficient,
            ElevatorConstants.elevatorDCoefficient,
            constraints
        );

        feedForward = new ElevatorFeedforward(-0.15, -0.18, 0.0, 0.0);

        // Set tolerances for PID controllers (if needed)
        profiledPIDControllerL.setTolerance(ElevatorConstants.positionTolerance);
        profiledPIDControllerR.setTolerance(ElevatorConstants.positionTolerance);
    }

    public void updatePose() {
        // Set the goal for the profiledPIDController
        profiledPIDControllerL.setGoal(elevatorState.position);
        profiledPIDControllerR.setGoal(elevatorState.position);

        // Get the PID controller output for both motors
        double pidOutputL = profiledPIDControllerL.calculate(encoderL.getPosition());
        double pidOutputR = profiledPIDControllerR.calculate(encoderR.getPosition());

        // Add feedforward compensation
        double feedforwardL = feedForward.calculate(encoderL.getPosition(), encoderL.getVelocity());
        double feedforwardR = feedForward.calculate(encoderR.getPosition(), encoderR.getVelocity());

        // Apply the outputs to the motors
        elevatorMotorL.set(pidOutputL + feedforwardL);
        elevatorMotorR.set(pidOutputR + feedforwardR);
    }

    public void setElevatorPosition(double position) {
        elevatorState.position = position;
    }

    public void setElevatorState(ElevatorState state) {
        elevatorState = state;
    }

    public ElevatorState getElevatorState() {
        return elevatorState;
    }

    public boolean hasReachedPose(double tolerance) {
        // Check if the current position is close to the goal position
        return Math.abs(encoderL.getPosition() - elevatorState.position) < tolerance && 
        Math.abs(encoderR.getPosition() - elevatorState.position) < tolerance;
     }

    public static elevator getInstance() {
        if (instance == null)
            instance = new elevator();
        return instance;
    }
}
