package frc.robot.commands.autons;

import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_L1_HEIGHT;
import static frc.robot.configs.constants.PhysicalConstants.Elevator.ELEVATOR_L2_HEIGHT;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Elevate;
import frc.robot.configs.constants.PhysicalConstants.Elevator;
import frc.robot.commands.GoToAprilTag;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

//This is a test to check if Go To April Tag will work for pathplanner
public class LimelightTest extends SequentialCommandGroup {
    public LimelightTest(CommandSwerveDrivetrain drivetrain, Vision vision, int tagID) {
        addCommands(
            new SequentialCommandGroup(
                new WaitCommand(0.2),
                new GoToAprilTag(drivetrain, vision, 0).withTimeout(5.0),
                new Elevate(ELEVATOR_L2_HEIGHT).withTimeout(2)
            )
        );
    }
}