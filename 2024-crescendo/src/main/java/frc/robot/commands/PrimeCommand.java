package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.MechanismState;

import frc.robot.Controls;
import frc.robot.MainSubsystemsManager;
import frc.robot.RangeTable;
import frc.robot.Robot;

public class PrimeCommand extends Command {
    private MainSubsystemsManager subsystemsManager;
    private double expectedRange;
    private Controls controls = new Controls();


    public PrimeCommand(MainSubsystemsManager subsystemsManager, double expectedRange){
        this.subsystemsManager = subsystemsManager;
        this.expectedRange = expectedRange;
        

    }
    @Override
    public void initialize() {
        controls = new Controls();
        
    }

    @Override
    public boolean execute() {
        if (!Robot.isReal()) {
            return true;
        }

        subsystemsManager.staticPrime(RangeTable.get(expectedRange));
        controls.kiddyPoolShot = true; //any shoot command enters prime
        subsystemsManager.updateMechanismStateMachine(controls, -1,true);

        if(subsystemsManager.mechanismState == MainSubsystemsManager.MechanismState.PRIMED){
            return true;
        }

        return false;
    }

    @Override
    public void shutdown() {
    
        
    }
    
}
