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
    private double bottomSensorTripTimeout = -1;
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
        if(shooter.state == Shooter.States.INTAKING || shooter.state == Shooter.States.RAM_TO_SHOOTERS){
            intake.setIntakeVelocity(INTAKE.INTAKE_VELOCITY);
        }
        else {
            intake.setIntakeVelocity(0);
        }
        if(!shooter.bottomBeamBreak.get()){ // Notice the exclamation point
            this.bottomSensorTripTimeout=-1; // This makes sure we don't time out if we catch the bottom beam break (successfully got a note) and then lose it
        }
        if(!Robot.isReal()){
            return true;
        }
        return shooter.state == Shooter.States.NOTHING // Waits until the note is fully in position
                || (this.timeoutSeconds != -1 && this.timeoutTimer.get() >= this.timeoutSeconds)
                || (this.bottomSensorTripTimeout != -1 && this.timeoutTimer.get() >= this.bottomSensorTripTimeout); // The beam breaks return the opposite of whether they're tripped
    }

    /**
     * The command will time-out after spending this long with nothing happening on the intake beam break.
     * @param timeout
     * @return
     */
    public IntakeCommand setBottomSensorTripTimeout(double timeout){
        this.bottomSensorTripTimeout = timeout;
        return this;
    }

    @Override
    public void shutdown() {
      intake.halt();
      shooter.stopFeeders();
    }
    
}
