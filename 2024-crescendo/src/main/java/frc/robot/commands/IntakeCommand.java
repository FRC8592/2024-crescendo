package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;
import frc.robot.MainSubsystemsManager.MechanismState;

public class IntakeCommand extends Command {
    private MainSubsystemsManager subsystemsManager;
    private double bottomSensorTripTimeout = -1;
    private Controls controls = new Controls();
    private enum States{
        WAIT,
        RUN
    }
    private States state = States.WAIT;

    public IntakeCommand(MainSubsystemsManager subsystemsManager){
        this.subsystemsManager = subsystemsManager;
    }


    @Override
    public void initialize() {
        controls = new Controls();
        this.timeoutTimer.start();
        controls.intake = true;
        controls.stow = false;
    }

    @Override
    public boolean execute() {
        Logger.recordOutput("CurrentCommand", "IntakeCommand");

        if(!Robot.isReal()){
            return true;
        }

        subsystemsManager.updateMechanismStateMachine(controls, 0, true);
        switch(state){
            case WAIT:
                if(subsystemsManager.mechanismState == MechanismState.INTAKING){
                    state = States.RUN;
                }
                break;
            case RUN:
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
                // no break; because it's unreachable and won't compile
        }
        return false;
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
