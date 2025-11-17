package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.RobotContainer;

//Mainly for teleop use, to hunt for and face an april tag while allowing forward/backward control
public class HuntTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final Joystick joystick;
    private final double maxRotSpeed;
    private final double maxForwardSpeed; 

    public HuntTag(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this(drivetrain, vision, new Joystick(0), 3.0, 2.0);
        addRequirements(drivetrain, vision);
    }

    public HuntTag(CommandSwerveDrivetrain drivetrain, Vision vision, Joystick joystick, double maxForwardSpeed, double maxRotSpeed) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.joystick = joystick;
        this.maxForwardSpeed = maxForwardSpeed;
        this.maxRotSpeed = maxRotSpeed;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() { 

    }

    @Override
    public void execute() {
        if (joystick.getRawButton(3)) {
            if (vision.hasTag()) {
                double[] data = vision.getPicturePosition();
                double xAngle = data[0];
                double rotCommand = Math.max(-maxRotSpeed, Math.min(maxRotSpeed, -xAngle * 0.03)); // P control 

                double forwardCommand = -joystick.getY() * maxForwardSpeed;

                drivetrain.drive(forwardCommand, 0, rotCommand);
            } else {
                drivetrain.drive(0, 0, 0); // stop if no tag
            }
        } else {
            drivetrain.drive(0, 0, 0); // stop if button not held
        }
    }

    @Override
    public boolean isFinished() {
        return false; // keep running while held
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0); // stop when command ends
    }
}
