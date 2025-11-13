package frc.robot.commands;

import static frc.robot.configs.constants.PIDconstants.Elevator.POS_CONSTRAINTS;
import static frc.robot.configs.constants.PIDconstants.Elevator.POS_TELEOP_CONSTRAINTS;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_AUTO_INTAKE_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_BARGE_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_FLOOR_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_HIGH_ALGAE_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_INTAKE_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_L1_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_L2_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_L3_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_L4_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_LOWER_LIMIT_SWITCH_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_LOW_ALGAE_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_POP_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_PROCESSOR_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_REST_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_UPPER_LIMIT_SWITCH_HEIGHT;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

//Did same thing for elevate (used revlib instead of evanlib), elevate lowkey a good name
public class Elevate extends Command {
    private final Elevator elevator;
    private final double targetPosition;

    private final double elevatorSpeed;
    private final double elevatorAcceleration;

    public Elevate(double targetPosition) {
        this.elevator = Elevator.getInstance();
        this.targetPosition = targetPosition;
        this.elevatorSpeed = POS_CONSTRAINTS.maxVelocity;
        this.elevatorAcceleration = POS_CONSTRAINTS.maxAcceleration;
        addRequirements(elevator);
    }

    public Elevate(double targetPosition, double elevatorSpeed) {
        this.elevator = Elevator.getInstance();
        this.targetPosition = targetPosition;
        this.elevatorSpeed = elevatorSpeed;
        this.elevatorAcceleration = POS_CONSTRAINTS.maxAcceleration;
        addRequirements(elevator);
    }

    public Elevate(double targetPosition, double elevatorSpeed, double elevatorAcceleration) {
        this.elevator = Elevator.getInstance();
        this.targetPosition = targetPosition;
        this.elevatorSpeed = elevatorSpeed;
        this.elevatorAcceleration = elevatorAcceleration;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.getPIDController().reset(elevator.getElevatorPositionInches());
        elevator.getPIDController().setConstraints(new Constraints(elevatorSpeed, elevatorAcceleration));
    }

    @Override
    public void execute() {
        elevator.setElevatorMotorPercent(
            -elevator.getPIDController().calculate(
                elevator.getElevatorPositionInches(),
                targetPosition
            )
        );
    }
    public void moveWithJoystick(double value) {
        // deadzone check
        if(Math.abs(value) < 0.05) {
            elevator.setElevatorMotorPercent(0);
            return;
        }

        double motorOutput = value * 0.5; // adjust max speed as needed
        elevator.setElevatorMotorPercent(motorOutput);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorMotorPercent(0);
    }

    public static Command l1() { return new Elevate(ELEVATOR_L1_HEIGHT); }
    public static Command l2() { return new Elevate(ELEVATOR_L2_HEIGHT); }
    public static Command l3() { return new Elevate(ELEVATOR_L3_HEIGHT); }
    public static Command l4() { return new Elevate(ELEVATOR_L4_HEIGHT); }

    public static Command teleopL4() {
        return new Elevate(
            ELEVATOR_L4_HEIGHT,
            POS_TELEOP_CONSTRAINTS.maxVelocity,
            POS_TELEOP_CONSTRAINTS.maxAcceleration
        );
    }

    public static Command intake() { return new Elevate(ELEVATOR_INTAKE_HEIGHT); }
    public static Command autoIntake() { return new Elevate(ELEVATOR_AUTO_INTAKE_HEIGHT); }
    public static Command zero() { return new Elevate(ELEVATOR_LOWER_LIMIT_SWITCH_HEIGHT); }
    public static Command max() { return new Elevate(-ELEVATOR_UPPER_LIMIT_SWITCH_HEIGHT); }
    public static Command rest() { return new Elevate(ELEVATOR_REST_HEIGHT); }
    public static Command lowAlgae() { return new Elevate(ELEVATOR_LOW_ALGAE_HEIGHT); }
    public static Command highAlgae() { return new Elevate(ELEVATOR_HIGH_ALGAE_HEIGHT); }
    public static Command pop() { return new Elevate(ELEVATOR_POP_HEIGHT); }
    public static Command processor() { return new Elevate(ELEVATOR_PROCESSOR_HEIGHT); }
    public static Command floor() { return new Elevate(ELEVATOR_FLOOR_HEIGHT); }
    public static Command barge() { return new Elevate(ELEVATOR_BARGE_HEIGHT); }
}
