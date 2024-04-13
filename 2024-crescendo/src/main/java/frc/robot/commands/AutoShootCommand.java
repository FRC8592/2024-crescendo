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
    }

    @Override
    public void initialize() {
        controls = new Controls();
        drive.drive(new ChassisSpeeds());

        /* Set up for an automatically ranged shot using vision */
        controls.rangeTableShoot = true;
        controls.score = true;
        subsystemsManager.setVisionPrime();   //turn on auto vision
    }
    @Override
    public boolean execute() {
        if (!Robot.isReal()) {
            return true;
        }

        /* Set up for an automatically ranged shot using vision */
        controls.rangeTableShoot = true;
        controls.score = true;
        subsystemsManager.setVisionPrime();   //turn on auto vision
            
        /* Rotate the robot to center the Apriltag fiducial in the camera view */
        double omega = vision.visual_servo(0, 3, APRILTAG_VISION.SPEAKER_AIM_TAGS, -1.5);
        drive.drive(new ChassisSpeeds(0, 0, omega));

        /* Update the subsystem manager with range */
        subsystemsManager.updateMechanismStateMachine(controls,
                vision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS),
                vision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS)<APRILTAG_VISION.X_ROT_LOCK_ERROR); //TODO put this in PoseVision


        /* When the shot is complete, disable our shooter controls and inform the autonomous subsystem that this command is complete*/
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
        subsystemsManager.updateMechanismStateMachine(controls,
            vision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS),
            vision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS)<APRILTAG_VISION.X_ROT_LOCK_ERROR); //TODO put this in PoseVision
    }
}
