package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.*;
import frc.robot.Constants.APRILTAG_LIMELIGHT;

import org.littletonrobotics.junction.Logger;


public class AutoAimCommand extends Command {
    private Swerve drive;
    private PoseVision vision;
    private double tolerance;

    public AutoAimCommand(Swerve drive, PoseVision vision, double tolerance) {
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
        Logger.recordOutput("CurrentCommand", "AutoAimCommand");
        double omega = vision.visual_servo(0, 3, 4, 0); //TODO: make sure this works on both sides with the tag ID
        drive.drive(new ChassisSpeeds(0, 0, omega));
        //Return false in leu of using the nonexistent getPositionRelativeToTag (or something like that) function
        return false;//(Math.abs(vision.processedDx)<APRILTAG_LIMELIGHT.LOCK_ERROR) && (Math.abs(vision.processedDy - APRILTAG_LIMELIGHT.SPEAKER_TY_TARGET) < APRILTAG_LIMELIGHT.CLOSE_ERROR); // && is target valid
    }
    @Override
    public void shutdown() {
        initialize();
    }
}
