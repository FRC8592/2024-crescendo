package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.Constants.*;
import frc.robot.MainSubsystemsManager.MechanismState;

public class ShootCommand extends Command{
    private MainSubsystemsManager subsystemsManager;
    private double expectedRange;
    private Timer timer = new Timer();
    private Controls controls = new Controls();
    private enum States{
        WAIT,
        RUN
    }
    private States state = States.WAIT;

    public ShootCommand(MainSubsystemsManager subsystemsManager, double expectedRange){
        this.subsystemsManager = subsystemsManager;
        this.expectedRange = expectedRange;
    }

    @Override
    public void initialize() {
        controls = new Controls();
        timer.reset();
        timeoutTimer.start();
        subsystemsManager.staticPrime(RangeTable.get(expectedRange));
        controls.score = true;
        state = States.WAIT;
    }

    @Override
    public boolean execute() {
        if (!Robot.isReal()) {
            return true;
        }

        Logger.recordOutput("CurrentCommand", "ShootCommand");
        controls.score = true;
        subsystemsManager.staticPrime(RangeTable.get(expectedRange));
      /*   switch(state){
            case WAIT:
                controls.score = true;
                subsystemsManager.updateMechanismStateMachine(controls, 0, true);
                if(subsystemsManager.mechanismState == MechanismState.PRIMING || subsystemsManager.mechanismState == MechanismState.PRIMED){
                    this.state = States.RUN;
                }
                return false;
            case RUN:
                subsystemsManager.updateMechanismStateMachine(controls, 0, true);
                return subsystemsManager.mechanismState == MechanismState.STOWING || (timeoutSeconds != -1 && timeoutTimer.get() >= timeoutSeconds);
        }
        return false;
        */
        subsystemsManager.updateMechanismStateMachine(controls, 0, true);
        return subsystemsManager.mechanismState == MechanismState.STOWING;
    }

    @Override
    public void shutdown() {
        controls.score = false;
        subsystemsManager.updateMechanismStateMachine(controls, 0, true);
    }
}
