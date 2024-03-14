package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.Constants.APRILTAG_LIMELIGHT;
import frc.robot.Constants.SHOOTER;

import org.littletonrobotics.junction.Logger;
import frc.robot.RangeTable.RangeEntry;


public class AutoShootCommand extends Command {
    private Swerve drive;
    private PoseVision vision;
    private Elevator elevator;
    private Shooter shooter;
    private Timer timer;


    public AutoShootCommand(Swerve drive, PoseVision vision, Elevator elevator, Shooter shooter) {
        this.drive = drive;
        this.vision = vision;
        this.elevator = elevator;
        this.shooter = shooter;
        this.timer = new Timer();
    }
    @Override
    public void initialize() {
        drive.drive(new ChassisSpeeds());
    }
    @Override
    public boolean execute() {
        Logger.recordOutput("CurrentCommand", "AutoShootCommand");
        double omega = vision.visual_servo(0, 3, 4, 0); //TODO: make sure this works on both sides with the tag ID
        Logger.recordOutput("AutoShootCommand Omega", omega);
        double distance = vision.distanceToAprilTag(4);
        /
        RangeEntry entry = RangeTable.get(distance);
        Logger.recordOutput("Distance to Tag 4", distance);
        elevator.setPivotAngleCustom(entry.pivotAngle);
        shooter.setShootVelocity(entry.flywheelSpeed, entry.flywheelSpeed);
        drive.drive(new ChassisSpeeds(0, 0, omega));
        if (Math.abs(vision.getCurrTagX())<APRILTAG_LIMELIGHT.LOCK_ERROR && shooter.isReady() && elevator.isTargetAngle()){
            SmartDashboard.putNumber("Ready To Shoot", distance);
            Logger.recordOutput("AutoShootCommand Shooting", true);
            this.timer.start();
            if(this.timer.get() < 0.1){
                shooter.setFeederVelocity(SHOOTER.OUTAKE_FEEDER_SPEED);
            }
            else if(this.timer.get() < SHOOTER.SHOOT_SCORE_TIME){
                shooter.setFeederVelocity(SHOOTER.SHOOTING_FEEDER_SPEED);
            }
            else{
                return true; // end the command
            }
        }
        else{
            Logger.recordOutput("AutoShootCommand Shooting", false);
        }
        return false;
    }

    public void shutdown() {
        drive.drive(new ChassisSpeeds());
        elevator.stow();
        shooter.stop();
        shooter.stopFeeders();
    }
}
