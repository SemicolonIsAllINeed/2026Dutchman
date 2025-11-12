package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

// Maily used for auton, prob make more limelight commands later
public class GoToAprilTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final int tagID;
    private boolean isFinished = false;

    public GoToAprilTag(CommandSwerveDrivetrain drivetrain, Vision vision, int tagID) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.tagID = tagID;
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        //Sometimes tag is sensed and immediately not sensed

        // if (!vision.hasTag()) {
        //     drivetrain.drive(0, 0, 0);
        //     return;
        // }

        Pose2d tagPose = vision.getFieldTagPose(tagID);
        Pose2d robotPose = drivetrain.getPose();

        double dx = tagPose.getX() - robotPose.getX();
        double dy = tagPose.getY() - robotPose.getY();

        double kP = 1.0; // Simple proportional gain
        double vx = kP * dx;
        double vy = kP * dy;

        // Limit max speed
        double maxSpeed = 2.0; // meters/sec
        vx = Math.max(-maxSpeed, Math.min(vx, maxSpeed));
        vy = Math.max(-maxSpeed, Math.min(vy, maxSpeed));

        drivetrain.drive(vx, vy, 0);

        if (Math.hypot(dx, dy) < 0.1) {
            isFinished = true;
            drivetrain.drive(0, 0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0);
    }
}
