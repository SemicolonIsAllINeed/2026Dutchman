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
        
        //Desired x & y locations minus actual
        double dx = tagPose.getX() - robotPose.getX();
        double dy = tagPose.getY() - robotPose.getY();
        
        double kP = 1.0; // Simple proportional gain
        double vx = kP * dx;
        double vy = kP * dy;

        // Limit max speed
        double maxSpeed = 2.0; //prob change for testing
        vx = Math.max(-maxSpeed, Math.min(vx, maxSpeed));
        vy = Math.max(-maxSpeed, Math.min(vy, maxSpeed));

        // angle between robot and tag
        double desiredAngle = Math.atan2(dy, dx);

        // current robot heading
        double currentAngle = robotPose.getRotation().getRadians();

        // rotation error
        double angleError = desiredAngle - currentAngle;

        //make in terms of -pi, pi
        angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError));

        // simple proportional rotation
        double kProt = 0.03;
        double rot = kProt * angleError;

        // limit rotation speed
        double maxRotSpeed = 2.5;
        rot = Math.max(-maxRotSpeed, Math.min(rot, maxRotSpeed));

        drivetrain.drive(vx, vy, rot);

        // finish when close enough AND facing tag, so it doesn't get too close and overshoot
        boolean positionGood = Math.hypot(dx, dy) < 0.1; // like 10 cm
        boolean rotationGood = Math.abs(angleError) < 0.08; // around 5 degrees

        if (positionGood && rotationGood) {
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
