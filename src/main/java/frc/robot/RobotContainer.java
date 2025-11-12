// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Elevate;
import frc.robot.commands.GoToAprilTag;
import frc.robot.commands.HuntTag;
import frc.robot.commands.autons.LimelightTest;
import frc.robot.commands.autons.Taxi;
import frc.robot.configs.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.03)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController xbox = new CommandXboxController(1);
    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandXboxController wController = new CommandXboxController(2);
    private final Vision vision = Vision.getInstance();
    private static SendableChooser<Command> autoChooser;




    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getY() * MaxSpeed * (((-joystick.getThrottle() + 1 ) / 2) + 0.1)) // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getX() * MaxSpeed * (((-joystick.getThrottle() + 1 ) / 2) + 0.1)) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getTwist() * MaxAngularRate * (((-joystick.getThrottle() + 1 ) / 2) + 0.1)) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()));
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        // reset the field-centric heading on left bumper press
        joystick.trigger().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);


        // Elevator controls
        Elevator elevator = Elevator.getInstance();
        wController.pov(90).onTrue(Elevate.rest());
        wController.pov(270).onTrue(Elevate.l2());
        wController.pov(180).onTrue(Elevate.l3());
        wController.pov(0).onTrue(Elevate.l4());

        //Safe control incase bad things happen
        Elevate elevatorCommand = new Elevate(0);
        new Trigger(() -> wController.getLeftY() > 0.05).whileTrue(Commands.run(() -> elevatorCommand.moveWithJoystick(wController.getLeftY())));
        new Trigger(() -> wController.getLeftY() < -0.05).whileTrue(Commands.run(() -> elevatorCommand.moveWithJoystick(wController.getLeftY())));
    
        //Hunt Tag - while holding button 3 on joystick should be able to move toward the april tag
        joystick.button(3).whileTrue(new HuntTag(drivetrain, vision));
    }
    public void configureAuto() {
        autoChooser = new SendableChooser<Command>();
		autoChooser.setDefaultOption("nothing", null);
		autoChooser.addOption("Timed Taxi", new Taxi());
        autoChooser.addOption("Limelight Test", new LimelightTest(drivetrain, vision, 0));
		SmartDashboard.putData("Auton Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public static CommandSwerveDrivetrain getSwerveDrivetrain() {
        return drivetrain;
    }
}
