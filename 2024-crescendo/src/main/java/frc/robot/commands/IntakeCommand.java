package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Intake;
import frc.robot.Robot;
import frc.robot.Shooter;

public class IntakeCommand extends Command {
    private Intake intake;
    private Shooter shooter;
    public IntakeCommand(Intake intake, Shooter shooter){
        this.intake = intake;
        this.shooter = shooter;
    }


    @Override
    public void initialize() {
        this.timer.start();
    }

    @Override
    public boolean execute() {
       //spins intake and feeder until beam sensor stops
        intake.spinPercentOutput(0.75);
        shooter.setFeederVelocity(4000);
        return shooter.hasNote() || (this.timeoutSeconds != -1 && this.timer.get() >= this.timeoutSeconds);
    }

    @Override
    public void shutdown() {
      intake.halt();
    }
    
}
