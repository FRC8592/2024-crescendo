package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;
import frc.robot.Constants.*;

public class AutoCollectCommand extends Command {
    LimelightTargeting targeting;
    Swerve drive;
    /**
     * MUST BE RUN WITH AN INTAKE COMMAND
     * @param targeting Limelight used to target the note
     * @param drive Drivetrain object to move
     * @param shooter Shooter object to detect whether we have a note
     */
    public AutoCollectCommand(LimelightTargeting targeting, Swerve drive) {
        this.targeting = targeting;
        this.drive = drive;
    }

    @Override
    public void initialize() {
        this.targeting.updateVision();
    }

    @Override
    public boolean execute() {
        this.targeting.updateVision();
        this.drive.drive(targeting.driveToTarget(
                new PIDController(NOTELOCK.DRIVE_TO_TURN_kP, NOTELOCK.DRIVE_TO_TURN_kI, NOTELOCK.DRIVE_TO_TURN_kD),
                new PIDController(NOTELOCK.DRIVE_TO_DRIVE_kP, NOTELOCK.DRIVE_TO_DRIVE_kI, NOTELOCK.DRIVE_TO_DRIVE_kD),
                NOTELOCK.DRIVE_TO_TARGET_ANGLE));
        return true;
    }

    @Override
    public void shutdown() {
    }
}
