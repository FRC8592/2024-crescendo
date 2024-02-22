package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Intake;
import frc.robot.Robot;
import frc.robot.Shooter;

public class IntakeCommand extends Command {
    private Intake intake;
    private Shooter shooter;
    private Timer timer = new Timer();
    public IntakeCommand(Intake intake, Shooter shooter){
        this.intake = intake;
        this.shooter = shooter;
    }


    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public boolean execute() {
       //spins intake and feeder until beam sensor stops
        intake.spinPercentOutput(0.75);
        shooter.setFeederVelocity(1000);
        if (Robot.isReal()) {
            return shooter.hasNote();
        } else {
            return timer.get() > 0.25;
        }
    }

    @Override
    public void shutdown() {
      intake.halt();
    }
    
}
