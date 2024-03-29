package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.Constants.APRILTAG_LIMELIGHT;
import frc.robot.Constants.APRILTAG_VISION;
import frc.robot.Constants.SHOOTER;

import org.littletonrobotics.junction.Logger;
import frc.robot.RangeTable.RangeEntry;


public class AutoShootCommand extends Command {
    private Swerve drive;
    private PoseVision vision;
    private Elevator elevator;
    private Shooter shooter;
    private Timer timer;
    private Timer aimTimeout;
    private boolean isShooting;


    public AutoShootCommand(Swerve drive, PoseVision vision, Elevator elevator, Shooter shooter) {
        this.drive = drive;
        this.vision = vision;
        this.elevator = elevator;
        this.shooter = shooter;
        this.timer = new Timer();
        this.aimTimeout = new Timer();
        this.isShooting = false;
    }
    @Override
    public void initialize() {
        drive.drive(new ChassisSpeeds());
        this.aimTimeout.start();
    }
    @Override
    public boolean execute() {
        Logger.recordOutput("CurrentCommand", "AutoShootCommand");
        double omega = vision.visual_servo(0, 3, APRILTAG_VISION.SPEAKER_AIM_TAGS, 0.5);
        Logger.recordOutput("AutoShootCommand Omega", omega);
        double distance = vision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS);
        RangeEntry entry = RangeTable.get(distance);
        Logger.recordOutput("Distance to Tag 4", distance);
        elevator.setPivotAngleCustom(entry.pivotAngle);
        shooter.setShootVelocity(entry.leftFlywheelSpeed, entry.leftFlywheelSpeed);
        // shooter.setShootVelocity(0, 0);
        drive.drive(new ChassisSpeeds(0, 0, omega));
        if ((Math.abs(vision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS))<APRILTAG_VISION.X_ROT_LOCK_ERROR 
            && elevator.isTargetAngle()) 
            || isShooting
            || aimTimeout.get() > 1.5){ // let it aim for 0.5 sec
            isShooting = true;

            // update range table shot params
            distance = vision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS);
            entry = RangeTable.get(distance);
            shooter.setShootVelocity(entry.leftFlywheelSpeed, entry.leftFlywheelSpeed);

            SmartDashboard.putNumber("Ready To Shoot", distance);
            Logger.recordOutput("AutoShootCommand Shooting", true);
            this.timer.start();
            if(this.timer.get() < SHOOTER.SHOOT_SCORE_TIME){
                if(shooter.readyToShoot()){
                    shooter.setFeederPower(SHOOTER.SHOOTING_FEEDER_POWER);
                }
                else{
                    shooter.stopFeeders();
                }
            }
            else{
                shooter.stopFeeders();
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
        shooter.stopFlywheels();
        shooter.stopFeeders();
    }
}
