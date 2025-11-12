package frc.robot.commands.autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.GoToAprilTag;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class LimelightTest extends SequentialCommandGroup {
    public LimelightTest(CommandSwerveDrivetrain drivetrain, Vision vision, int tagID) {
        addCommands(
            new SequentialCommandGroup(
                new WaitCommand(1),
                new GoToAprilTag(drivetrain, vision, tagID).withTimeout(5.0)
                )
        );
    }
}