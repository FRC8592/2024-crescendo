package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;
import frc.robot.Constants.*;

public class AutoCollectCommand extends Command {
    LimelightTargeting targeting;
    Swerve drive;
    Shooter shooter;
    /**
     * MUST BE RUN WITH AN INTAKE COMMAND
     * @param targeting Limelight used to target the note
     * @param drive Drivetrain object to move
     * @param shooter Shooter object to detect whether we have a note
     */
    public AutoCollectCommand(LimelightTargeting targeting, Swerve drive, Shooter shooter) {
        this.targeting = targeting;
        this.drive = drive;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        this.targeting.updateVision();
    }

    @Override
    public boolean execute() {
        // Logger.recordOutput("CurrentCommand", "AutoCollectCommand");
        // Logger.recordOutput("hasNote", shooter.hasNote);

        // this.targeting.updateVision();
        // this.drive.drive(targeting.driveToTarget(
        //         new PIDController(NOTELOCK.DRIVE_TO_TURN_kP, NOTELOCK.DRIVE_TO_TURN_kI, NOTELOCK.DRIVE_TO_TURN_kD),
        //         new PIDController(NOTELOCK.DRIVE_TO_DRIVE_kP, NOTELOCK.DRIVE_TO_DRIVE_kI, NOTELOCK.DRIVE_TO_DRIVE_kD),
        //         NOTELOCK.AUTO_DRIVE_TO_TARGET_ANGLE));
        // return !this.targeting.isTargetValid() || shooter.hasNote;
        return true; //TODO: If we want to use this in the future, it needs to be updated to not depend on shooter.hasNote
    }

    @Override
    public void shutdown() {
        drive.drive(new ChassisSpeeds());
    }
}
