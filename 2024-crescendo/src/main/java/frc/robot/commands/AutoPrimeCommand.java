package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.MechanismState;

import frc.robot.Controls;
import frc.robot.MainSubsystemsManager;
import frc.robot.PoseVision;
import frc.robot.RangeTable;
import frc.robot.Robot;
import frc.robot.Constants.APRILTAG_VISION;

public class AutoPrimeCommand extends Command {
    private MainSubsystemsManager subsystemsManager;
    private Controls controls = new Controls();
    private PoseVision vision;


    public AutoPrimeCommand(MainSubsystemsManager subsystemsManager, PoseVision vision){
        this.subsystemsManager = subsystemsManager;
        this.vision = vision;

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

        subsystemsManager.setVisionPrime();
        controls.rangeTableShoot = true;
        subsystemsManager.updateMechanismStateMachine(controls, 
                vision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS),
                vision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS) < APRILTAG_VISION.X_ROT_LOCK_ERROR);


        return true;
    }

    @Override
    public void shutdown() {
    
        
    }
    
}
