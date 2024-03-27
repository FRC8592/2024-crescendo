package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Intake;
import frc.robot.MainSubsystemsManager;
import frc.robot.Robot;
import frc.robot.Shooter;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.SHOOTER;
import frc.robot.MainSubsystemsManager.MechanismState;

public class IntakeCommand extends Command {
    private MainSubsystemsManager subsystemsManager;
    private double bottomSensorTripTimeout = -1;
    public IntakeCommand(MainSubsystemsManager subsystemsManager){
        this.subsystemsManager = subsystemsManager;
    }


    @Override
    public void initialize() {
        this.timeoutTimer.start();
        subsystemsManager.setState(MechanismState.INTAKING);
    }

    @Override
    public boolean execute() {
        Logger.recordOutput("CurrentCommand", "IntakeCommand");

        if(!Robot.isReal()){
            return true;
        }

        if(subsystemsManager.shooter.isBottomBeamBreakTripped()){
           this.bottomSensorTripTimeout=-1; // This makes sure we don't time out if we catch the bottom beam break (successfully got a note) and then lose it
        }

        if(subsystemsManager.mechanismState == MechanismState.LOADED){
            return true;
        }
        else{
            return (this.timeoutSeconds != -1 && this.timeoutTimer.get() >= this.timeoutSeconds)
                    || (this.bottomSensorTripTimeout != -1 && this.timeoutTimer.get() >= this.bottomSensorTripTimeout);
        }
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
    public void shutdown() {}
}
