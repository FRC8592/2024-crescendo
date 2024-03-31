package frc.robot.commands;

import frc.robot.Controls;
import frc.robot.MainSubsystemsManager;

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
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean execute() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void shutdown() {
        // TODO Auto-generated method stub
        
    }
    
}
