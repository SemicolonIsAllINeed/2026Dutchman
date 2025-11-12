package frc.robot.subsystems;

import static frc.robot.configs.constants.PIDconstants.Elevator.*;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.*;
import static frc.robot.configs.constants.PhysicalConstants.*;
import static frc.robot.configs.constants.PortConstants.Elevator.*;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Uhh Elevator subsystem using REVLib no library it no works (org.mort11)
public class Elevator extends SubsystemBase {
    private static Elevator instance;

    private final SparkMax motor;
    private final SparkAbsoluteEncoder absEncoder;
    private final SparkClosedLoopController closedLoop;

    private final DigitalInput lowerLimitSwitch;
    private final DigitalInput upperLimitSwitch;

    private double motorSpeed;
    private double elevatorPosition;
    private double rotationsCompleted;

    private final ProfiledPIDController controller;
    private final ElevatorFeedforward feedforward;

    private Elevator() {
        // Initialize motor
        motor = new SparkMax(MOTOR, MotorType.kBrushless);

        // REVLib use
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        // Configure encoder (absolute)
        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(ROTATIONS_TO_INCHES);
        encoderConfig.velocityConversionFactor(ROTATIONS_TO_INCHES);

        // Configure closed-loop
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();
        pidConfig.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        // Apply configs...
        motor.configure(
            config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters
        );

        absEncoder = motor.getAbsoluteEncoder();
        closedLoop = motor.getClosedLoopController();

        lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_SWITCH);
        upperLimitSwitch = new DigitalInput(UPPER_LIMIT_SWITCH);

        controller = new ProfiledPIDController(
            POS_KP,
            POS_KI,
            POS_KD,
            new Constraints(POS_CONSTRAINTS.maxVelocity, POS_CONSTRAINTS.maxAcceleration)
        );

        feedforward = new ElevatorFeedforward(POS_KS, POS_KG, POS_KV, POS_KA);

        motorSpeed = 0;
        elevatorPosition = 0;
        rotationsCompleted = ELEVATOR_OFFSET / ROTATIONS_TO_INCHES;
    }

    @Override
    public void periodic() {
        motor.setVoltage(motorSpeed * ROBOT_VOLTAGE);

        elevatorPosition = calculateElevatorPosition();
        fixWithLimitSwitch();

        SmartDashboard.putNumber("Elevator Height (in)", getElevatorPositionInches());
        SmartDashboard.putNumber("Elevator Speed (in/s)", getElevatorVelocityInches());
        SmartDashboard.putBoolean("Lower Limit", getAtLowerLimitSwitch());
        SmartDashboard.putBoolean("Upper Limit", getAtUpperLimitSwitch());
    }

    // Set elevator output percentage 
    public void setElevatorMotorPercent(double motorPercent) {
        this.motorSpeed = motorPercent + POS_KG;
    }

    public void setElevatorPosition(double newPoseInches) {
        rotationsCompleted -= (getElevatorPositionInches() + newPoseInches) / ROTATIONS_TO_INCHES;
        elevatorPosition = (getAbsoluteEncoderPositionRotations() + rotationsCompleted) * ROTATIONS_TO_INCHES;
    }

    private void fixWithLimitSwitch() {
        if (getAtLowerLimitSwitch()) {
            setElevatorPosition(ELEVATOR_LOWER_LIMIT_SWITCH_HEIGHT);
        }
        if (getAtUpperLimitSwitch()) {
            setElevatorPosition(ELEVATOR_UPPER_LIMIT_SWITCH_HEIGHT);
        }
    }

    public double getElevatorPositionInches() {
        return elevatorPosition;
    }

    public double getElevatorVelocityInches() {
        return absEncoder.getVelocity();
    }

    public double getAbsoluteEncoderPositionRotations() {
        return absEncoder.getPosition();
    }

    public ProfiledPIDController getPIDController() {
        return controller;
    }

    private double calculateElevatorPosition() {
        double inchesFound = (getAbsoluteEncoderPositionRotations() + rotationsCompleted) * ROTATIONS_TO_INCHES;
        if ((inchesFound - elevatorPosition) > MAXIMUM_INCH_CHANGE) {
            rotationsCompleted -= 1;
        }
        if ((elevatorPosition - inchesFound) > MAXIMUM_INCH_CHANGE) {
            rotationsCompleted += 1;
        }
        return (getAbsoluteEncoderPositionRotations() + rotationsCompleted) * ROTATIONS_TO_INCHES;
    }

    public boolean getAtLowerLimitSwitch() {
        return !lowerLimitSwitch.get();
    }

    public boolean getAtUpperLimitSwitch() {
        return !upperLimitSwitch.get();
    }

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }
}