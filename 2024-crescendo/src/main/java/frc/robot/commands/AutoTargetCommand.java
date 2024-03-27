package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.*;
import frc.robot.Constants.APRILTAG_LIMELIGHT;
import frc.robot.Constants.APRILTAG_VISION;

import org.littletonrobotics.junction.Logger;


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
        // Logger.recordOutput("CurrentCommand", "AutoTargetCommand");
        // double omega = vision.visual_servo(0, 3, APRILTAG_VISION.SPEAKER_AIM_TAGS, 0);
        // double vy = vision.visual_servo(0, 3, APRILTAG_VISION.SPEAKER_AIM_TAGS, 0);
        // drive.drive(new ChassisSpeeds(vy, 0, omega));
        
        return true; //TODO: Delete this or update it with our newer code
    }
    @Override
    public void shutdown() {
        initialize();
    }
}
