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
    }

    @Override
    public void initialize() {
        timer.reset();
        timeoutTimer.start();
        subsystemsManager.staticPrime(RangeTable.get(expectedRange));
    }

    @Override
    public boolean execute() {
        switch(state){
            case WAIT:
                controls.score = true;
                subsystemsManager.updateMechanismStateMachine(controls, 0, true);
                if(subsystemsManager.mechanismState == MechanismState.PRIMING){
                    this.state = States.RUN;
                }
                return false;
            case RUN:
                Logger.recordOutput("CurrentCommand", "ShootCommand");

                Robot.currentRange = RangeTable.get(expectedRange);

                return subsystemsManager.mechanismState == MechanismState.STOWING || (timeoutSeconds != -1 && timeoutTimer.get() >= timeoutSeconds);
        }
        return false;
    }

    @Override
    public void shutdown() {}
}
