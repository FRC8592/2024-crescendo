package frc.robot.commands;

import com.fasterxml.jackson.core.format.MatchStrength;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.Constants.*;

import frc.robot.MainSubsystemsManager.MechanismState;

import org.littletonrobotics.junction.Logger;
import frc.robot.RangeTable.RangeEntry;


public class AutoShootCommand extends Command {
    private Swerve drive;
    private PoseVision vision;
    private MainSubsystemsManager subsystemsManager;
    private Timer timer;
    private Controls controls;
    private enum States{
        WAIT,
        RUN
    }
    private States state = States.WAIT;


    public AutoShootCommand(Swerve drive, PoseVision vision, MainSubsystemsManager subsystemsManager) {
        this.drive = drive;
        this.vision = vision;
        this.subsystemsManager = subsystemsManager;
        this.timer = new Timer();
        this.controls = new Controls();
    }
    @Override
    public void initialize() {
        controls = new Controls();
        drive.drive(new ChassisSpeeds());
        controls.rangeTableShoot = true;
        subsystemsManager.setVisionPrime();   //turn on auto vision
    }
    @Override
    public boolean execute() {
        double omega = vision.visual_servo(0, 3, APRILTAG_VISION.SPEAKER_AIM_TAGS, 0.5);
        drive.drive(new ChassisSpeeds(0, 0, omega));
        subsystemsManager.updateMechanismStateMachine(controls,
                vision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS),
                vision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS)<APRILTAG_VISION.X_ROT_LOCK_ERROR); //TODO put this in PoseVision
        // switch(this.state){
        //     case WAIT:
                // if(subsystemsManager.mechanismState == MechanismState.PRIMING || subsystemsManager.mechanismState == MechanismState.PRIMED){
                //     this.state = States.RUN;
                // }
                // break;
            // case RUN:
            if (subsystemsManager.mechanismState == MechanismState.PRIMING){
                controls.rangeTableShoot = false;
            }
                if (subsystemsManager.mechanismState == MechanismState.PRIMED){
                    controls.score = true;
                }

                if(subsystemsManager.mechanismState == MechanismState.STOWING){
                    return true;
                }

                Logger.recordOutput("CurrentCommand", "AutoShootCommand");
                Logger.recordOutput("AutoShootCommand Omega", omega);
        // }

        return false;
    }

    public void shutdown() {
        drive.drive(new ChassisSpeeds());
    }
}
