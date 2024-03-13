package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.*;
import org.littletonrobotics.junction.Logger;


public class AutoTargetCommand extends Command {
    private Swerve drive;
    private PoseVision vision;
    private double tolerance;

    public AutoTargetCommand(Swerve drive, PoseVision vision, double tolerance) {
        this.drive = drive;
        this.vision = vision;
        this.tolerance = tolerance;
    }
    @Override
    public void initialize() {
        drive.drive(new ChassisSpeeds());
    }
    @Override
    public boolean execute() {
        Logger.recordOutput("CurrentCommand", "AutoTargetCommand");
        double omega = vision.visual_servo(0, 3, 4, 0);
        double vy = vision.visual_servo(0, 3, 4, 0);
        drive.drive(new ChassisSpeeds(vy, 0, omega));
        //return false for now, since TODO there's no method to get position relative to a specified ID
        return false;//(Math.abs(vision.processedDx)<APRILTAG_LIMELIGHT.LOCK_ERROR) && (Math.abs(vision.processedDy - APRILTAG_LIMELIGHT.SPEAKER_TY_TARGET) < APRILTAG_LIMELIGHT.CLOSE_ERROR); // && is target valid
    }
    @Override
    public void shutdown() {
        initialize();
    }
}
