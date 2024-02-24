package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Intake;
import frc.robot.Robot;
import frc.robot.Shooter;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.SHOOTER;

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
    }

    @Override
    public boolean execute() {
       //spins intake and feeder until beam sensor stops
        intake.spinPercentOutput(INTAKE.INTAKE_POWER);
        shooter.setFeederVelocity(SHOOTER.INTAKE_FEEDER_SPEED);
        timer.start();
        if (Robot.isReal()) {
            return shooter.hasNote();
        } else {
            return timer.get() > 0.25;
        }
    }

    @Override
    public void shutdown() {
      intake.halt();
      shooter.stopFeeders();
    }
    
}
