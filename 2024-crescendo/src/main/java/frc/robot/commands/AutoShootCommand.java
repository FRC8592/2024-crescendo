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
    private boolean isShooting;


    public AutoShootCommand(Swerve drive, PoseVision vision, MainSubsystemsManager subsystemsManager) {
        this.drive = drive;
        this.vision = vision;
        this.subsystemsManager = subsystemsManager;
        this.timer = new Timer();
    }
    @Override
    public void initialize() {
        drive.drive(new ChassisSpeeds());
        subsystemsManager.setState(MechanismState.PRIMING);
    }
    @Override
    public boolean execute() {
        double omega = vision.visual_servo(0, 3, APRILTAG_VISION.SPEAKER_AIM_TAGS, 0.5);
        double distance = vision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS);
        Robot.currentRange = RangeTable.get(distance);
        drive.drive(new ChassisSpeeds(0, 0, omega));

        if ((Math.abs(vision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS))<APRILTAG_VISION.X_ROT_LOCK_ERROR
                && subsystemsManager.mechanismState == MechanismState.PRIMED)){
            subsystemsManager.setState(MechanismState.SHOOTING);
        }
        else{
            if(subsystemsManager.mechanismState == MechanismState.STOWING){
                return true;
            }
        }

        Logger.recordOutput("CurrentCommand", "AutoShootCommand");
        Logger.recordOutput("AutoShootCommand Omega", omega);
        Logger.recordOutput("Distance to Tag 4", distance);

        return false;
    }

    public void shutdown() {
        drive.drive(new ChassisSpeeds());
    }
}
