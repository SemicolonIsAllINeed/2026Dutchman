package frc.robot.commands.autons;

import frc.robot.commands.Elevate;
import frc.robot.commands.GoToAprilTag;
import frc.robot.commands.HuntTag;
import frc.robot.configs.constants.PhysicalConstants;
import frc.robot.RobotContainer;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class BasicCommands {
    public static CommandSwerveDrivetrain drivetrain;
    public static Vision vision;
        
        public static void setCommands() {
            NamedCommands.registerCommand("L2", Elevate.l2());
            NamedCommands.registerCommand("L3", Elevate.l3());
            NamedCommands.registerCommand("L4", Elevate.l4());
            NamedCommands.registerCommand("Rest", Elevate.rest());
            NamedCommands.registerCommand("Intake", Elevate.intake());
            NamedCommands.registerCommand("GoLimelight", new GoToAprilTag(drivetrain, vision, 0).withTimeout(5.0));    
    }
}
