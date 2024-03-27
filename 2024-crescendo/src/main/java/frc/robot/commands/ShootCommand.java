package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Elevator;
import frc.robot.MainSubsystemsManager;
import frc.robot.RangeTable;
import frc.robot.Robot;
import frc.robot.Shooter;
import frc.robot.Constants.*;
import frc.robot.MainSubsystemsManager.MechanismState;

public class ShootCommand extends Command{
    private MainSubsystemsManager subsystemsManager;
    private double expectedRange;
    private Timer timer = new Timer();

    public ShootCommand(MainSubsystemsManager subsystemsManager, double expectedRange){
        this.subsystemsManager = subsystemsManager;
    }

    @Override
    public void initialize() {
        timer.reset();
        subsystemsManager.setState(MechanismState.PRIMING);
        timeoutTimer.start();
    }

    @Override
    public boolean execute() {
        Logger.recordOutput("CurrentCommand", "ShootCommand");

        Robot.currentEntry = RangeTable.get(expectedRange);

        if(subsystemsManager.mechanismState == MechanismState.PRIMED){
            subsystemsManager.setState(MechanismState.SHOOTING);
        }

        return subsystemsManager.mechanismState == MechanismState.STOWING || (timeoutSeconds != -1 && timeoutTimer.get() >= timeoutSeconds);
    }

    @Override
    public void shutdown() {}
}
