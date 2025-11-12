package frc.robot.commands.autons;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
public class TimedDrive extends Command{
    private CommandSwerveDrivetrain drivetrain;
    private Timer timer;
    private double time;
    private double x;
    private double y;
    private double omega;
    //Field orientated is not neccesary since auton will always start with the robot field orientated, according to Ms. Varner
    //X, Y, & Omega are the x, y, and angular velocities respectively
    public TimedDrive(double time, double x, double y, double omega){
        
        this.time = time;
        this.x = x;
        this.y = y;
        this.omega = omega;

        drivetrain = RobotContainer.getSwerveDrivetrain();

        timer = new Timer();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        drivetrain.driveRelative(x, y, omega);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.driveRelative(0,0,0);
    }

    @Override
    public boolean isFinished(){
        return timer.get() > time;
    }
}
