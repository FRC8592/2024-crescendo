package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.*;
import frc.robot.Constants.APRILTAG_LIMELIGHT;
import org.littletonrobotics.junction.Logger;


public class AutoAimCommand extends Command {
    private PIDController turnPID;
    private PIDController drivePID;
    private Swerve drive;
    private LimelightTargeting targeting;
    private double tolerance;

    public AutoAimCommand(Swerve drive, LimelightTargeting vision, double tolerance) {
        turnPID = new PIDController(APRILTAG_LIMELIGHT.SPEAKER_TURN_kP, APRILTAG_LIMELIGHT.SPEAKER_TURN_kI, APRILTAG_LIMELIGHT.SPEAKER_TURN_kD);
        drivePID = new PIDController(APRILTAG_LIMELIGHT.SPEAKER_DRIVE_kP, APRILTAG_LIMELIGHT.SPEAKER_DRIVE_kI, APRILTAG_LIMELIGHT.SPEAKER_DRIVE_kD);
        this.drive = drive;
        this.targeting = vision;
        this.tolerance = tolerance;
        turnPID.setTolerance(tolerance);
    }
    @Override
    public void initialize() {
        drive.drive(new ChassisSpeeds());
    }
    @Override
    public boolean execute() {
        Logger.recordOutput("CurrentCommand", "AutoAimCommand");

        targeting.updateVision();
        double omega = targeting.turnRobot(0.0, turnPID, "tx", 2.0, 0.0);
        double vy = targeting.turnRobot(0, drivePID, "ty", 2.0, APRILTAG_LIMELIGHT.SPEAKER_TY_TARGET);
        drive.drive(new ChassisSpeeds(0, vy,
            omega));
        return (omega == 0); // && is target valid
    }
    @Override
    public void shutdown() {
        initialize();
    }
}
