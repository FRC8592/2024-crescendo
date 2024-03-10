package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.*;
import frc.robot.Constants.APRILTAG_LIMELIGHT;
import org.littletonrobotics.junction.Logger;
import frc.robot.RangeTable.RangeEntry;


public class AutoShootCommand extends Command {
    private Swerve drive;
    private PoseVision vision;
    private Elevator elevator;
    private Shooter shooter;


    public AutoShootCommand(Swerve drive, PoseVision vision, Elevator elevator, Shooter shooter) {
        this.drive = drive;
        this.vision = vision;
        this.elevator = elevator;
        this.shooter = shooter;
    }
    @Override
    public void initialize() {
        drive.drive(new ChassisSpeeds());
    }
    @Override
    public boolean execute() {
        Logger.recordOutput("CurrentCommand", "AutoShootCommand");
        double omega = vision.visual_servo(0, 3, 4, 0); //TODO: make sure this works on both sides with the tag ID
        RangeEntry entry = RangeTable.get(vision.getCurrTagZ());
        elevator.setPivotAngleCustom(entry.pivotAngle);
        shooter.setShootVelocity(entry.flywheelSpeed, entry.flywheelSpeed);
        drive.drive(new ChassisSpeeds(0, 0, omega));
        return (Math.abs(vision.getCurrTagX())<APRILTAG_LIMELIGHT.LOCK_ERROR); // && is target valid
    }
    @Override
    public void shutdown() {
        initialize();
    }
}
