package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.*;
import frc.robot.Constants.APRILTAG_LIMELIGHT;
import frc.robot.PoseVision.TargetVariable;

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
        double omega = vision.target(TargetVariable.LEFT_RIGHT_ROTATION, 3, 4, 0);
        drive.drive(new ChassisSpeeds(0, 0, omega));
        //Return false in leu of using the nonexistent getPositionRelativeToTag (or something like that) function
        return false;//(Math.abs(vision.processedDx)<APRILTAG_LIMELIGHT.LOCK_ERROR) && (Math.abs(vision.processedDy - APRILTAG_LIMELIGHT.SPEAKER_TY_TARGET) < APRILTAG_LIMELIGHT.CLOSE_ERROR); // && is target valid
    }
    @Override
    public void shutdown() {
        initialize();
    }
}
