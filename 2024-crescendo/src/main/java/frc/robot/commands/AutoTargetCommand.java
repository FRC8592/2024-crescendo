package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.*;
import frc.robot.Constants.APRILTAG_LIMELIGHT;
import org.littletonrobotics.junction.Logger;
import frc.robot.PoseVision.TargetVariable;


public class AutoTargetCommand extends Command {
    private PIDController turnPID;
    private PIDController drivePID;
    private Swerve drive;
    private PoseVision vision;
    private double tolerance;

    public AutoTargetCommand(Swerve drive, PoseVision vision, double tolerance) {
        turnPID = new PIDController(APRILTAG_LIMELIGHT.SPEAKER_TURN_kP, APRILTAG_LIMELIGHT.SPEAKER_TURN_kI, APRILTAG_LIMELIGHT.SPEAKER_TURN_kD);
        drivePID = new PIDController(APRILTAG_LIMELIGHT.SPEAKER_DRIVE_kP, APRILTAG_LIMELIGHT.SPEAKER_DRIVE_kI, APRILTAG_LIMELIGHT.SPEAKER_DRIVE_kD);
        this.drive = drive;
        this.vision = vision;
        this.tolerance = tolerance;
        turnPID.setTolerance(tolerance);
        turnPID.setIZone(APRILTAG_LIMELIGHT.SPEAKER_TURN_IZONE);
    }
    @Override
    public void initialize() {
        drive.drive(new ChassisSpeeds());
    }
    @Override
    public boolean execute() {
        Logger.recordOutput("CurrentCommand", "AutoTargetCommand");
        double omega = vision.target(TargetVariable.LEFT_RIGHT_ROTATION, 3/*TODO: I'll assume rad/frame for now, but I have no idea. It could be deg/frame*/, 
                4, 0);
        double vy = vision.target(TargetVariable.FORWARD_BACK_POSITION, 3/*TODO: Again, no idea on the units. Probably in/frame*/, 
                4, 0);
        drive.drive(new ChassisSpeeds(vy, 0, omega));
        //return false for now, since TODO there's no method to get position relative to a specified ID
        return false;//(Math.abs(vision.processedDx)<APRILTAG_LIMELIGHT.LOCK_ERROR) && (Math.abs(vision.processedDy - APRILTAG_LIMELIGHT.SPEAKER_TY_TARGET) < APRILTAG_LIMELIGHT.CLOSE_ERROR); // && is target valid
    }
    @Override
    public void shutdown() {
        initialize();
    }
}
