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
    private boolean isShooting;


    public AutoShootCommand(Swerve drive, PoseVision vision, Elevator elevator, Shooter shooter) {
        this.drive = drive;
        this.vision = vision;
        this.elevator = elevator;
        this.shooter = shooter;
        this.timer = new Timer();
        this.isShooting = false;
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
        RangeEntry entry = RangeTable.get(distance);
        Logger.recordOutput("Distance to Tag 4", distance);
        elevator.setPivotAngleCustom(entry.pivotAngle);
        // shooter.setShootVelocity(0, 0);
        drive.drive(new ChassisSpeeds(0, 0, omega));
        if ((Math.abs(vision.getCurrTagX())<APRILTAG_LIMELIGHT.LOCK_ERROR && elevator.isTargetAngle()) || isShooting){
            isShooting = true;
            SmartDashboard.putNumber("Ready To Shoot", distance);
            Logger.recordOutput("AutoShootCommand Shooting", true);
            this.timer.start();
            if(this.timer.get() < 0.05){
                shooter.setFeederVelocity(SHOOTER.OUTAKE_FEEDER_SPEED);
            }
            else if(this.timer.get() < SHOOTER.SHOOT_SCORE_TIME){
                shooter.setShootVelocity(entry.flywheelSpeed, entry.flywheelSpeed);
                if(shooter.readyToShoot()){
                    shooter.setFeederVelocity(SHOOTER.SHOOTING_FEEDER_SPEED);
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
        shooter.stop();
        shooter.stopFeeders();
    }
}
