package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Intake;
import frc.robot.Robot;
import frc.robot.Shooter;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.SHOOTER;

public class IntakeCommand extends Command {
    private Intake intake;
    private Shooter shooter;
    public IntakeCommand(Intake intake, Shooter shooter){
        this.intake = intake;
        this.shooter = shooter;
    }


    @Override
    public void initialize() {
        this.timeoutTimer.start();
        shooter.intake();
    }

    @Override
    public boolean execute() {
        Logger.recordOutput("CurrentCommand", "IntakeCommand");

       //spins intake and feeder until beam sensor stops
        intake.setIntakeVelocity(INTAKE.INTAKE_VELOCITY);
        if(!Robot.isReal()){
            return true;
        }
        return shooter.state == Shooter.IntakeStates.NOTHING // Waits until the note is fully in position
                || (this.timeoutSeconds != -1 && this.timeoutTimer.get() >= this.timeoutSeconds);
    }

    @Override
    public void shutdown() {
      intake.halt();
      shooter.stopFeeders();
    }
    
}
