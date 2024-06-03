// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.*;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import frc.robot.helpers.*;

public class Shooter extends NewtonSubsystem {
    private SparkFlexControl leftShooterMotor;
    private SparkFlexControl rightShooterMotor;
    private SparkFlexControl feederMotor;

    private DigitalInput bottomBeamBreak;
    private DigitalInput topBeamBreak;
    private DigitalInput middleBeamBreak;

    // Only used for logging
    private int leftTargetSpeed = 0;
    private int rightTargetSpeed = 0;

    public Shooter() {
        leftShooterMotor = new SparkFlexControl(CAN.TOP_SHOOTER_MOTOR_CAN_ID, false);
        leftShooterMotor.setPIDF(SHOOTER.LEFT_SHOOTER_MOTOR_kP,  SHOOTER.LEFT_SHOOTER_MOTOR_kI,  SHOOTER.LEFT_SHOOTER_MOTOR_kD,  SHOOTER.LEFT_SHOOTER_MOTOR_kF,  0);
        leftShooterMotor.motorControl.setIZone(SHOOTER.SHOOTER_MOTOR_IZONE);
        leftShooterMotor.setCurrentLimit (POWER.LEFT_SHOOTER_MOTOR_CURRENT_LIMIT,  POWER.LEFT_SHOOTER_MOTOR_CURRENT_LIMIT);
        leftShooterMotor.setInverted();
        leftShooterMotor.setPercentOutput(0);

        rightShooterMotor = new SparkFlexControl(CAN.BOTTOM_SHOOTER_MOTOR_CAN_ID, false);
        rightShooterMotor.setPIDF(SHOOTER.RIGHT_SHOOTER_MOTOR_kP, SHOOTER.RIGHT_SHOOTER_MOTOR_kI, SHOOTER.RIGHT_SHOOTER_MOTOR_kD, SHOOTER.RIGHT_SHOOTER_MOTOR_kF, 0);
        rightShooterMotor.setCurrentLimit(POWER.RIGHT_SHOOTER_MOTOR_CURRENT_LIMIT, POWER.RIGHT_SHOOTER_MOTOR_CURRENT_LIMIT);
        rightShooterMotor.setPercentOutput(0);
        rightShooterMotor.motorControl.setIZone(SHOOTER.SHOOTER_MOTOR_IZONE);

        feederMotor = new SparkFlexControl(CAN.FEEDER_MOTOR_CAN_ID, false);
        feederMotor.setPIDF(SHOOTER.FEEDER_MOTOR_LOADED_kP, SHOOTER.FEEDER_MOTOR_LOADED_kI, SHOOTER.FEEDER_MOTOR_LOADED_kD, SHOOTER.FEEDER_MOTOR_LOADED_kF, 1);
        feederMotor.setPIDF(SHOOTER.FEEDER_MOTOR_OUTAKE_kP, SHOOTER.FEEDER_MOTOR_OUTAKE_kI, SHOOTER.FEEDER_MOTOR_OUTAKE_kD, SHOOTER.FEEDER_MOTOR_OUTAKE_kF, 2);
        feederMotor.setPIDF(SHOOTER.FEEDER_MOTOR_kP,SHOOTER.FEEDER_MOTOR_kI,SHOOTER.FEEDER_MOTOR_kD,SHOOTER.FEEDER_MOTOR_kF,0);
        feederMotor.motorControl.setIZone(SHOOTER.FEEDER_MOTOR_LOADED_IZONE, 1);
        feederMotor.motorControl.setIZone(SHOOTER.FEEDER_MOTOR_OUTAKE_IZONE, 2);
        feederMotor.setCurrentLimit(POWER.FEEDER_MOTOR_CURRENT_LIMIT,POWER.FEEDER_MOTOR_CURRENT_LIMIT);
        feederMotor.setInverted();
        feederMotor.setPercentOutput(0);

        bottomBeamBreak = new DigitalInput(SHOOTER.BOTTOM_BEAM_BREAK_DIO_PORT);
        topBeamBreak = new DigitalInput   (SHOOTER.TOP_BEAM_BREAK_DIO_PORT);
        middleBeamBreak = new DigitalInput(SHOOTER.MIDDLE_BEAM_BREAK_DIO_PORT);
    }


    /**
     * Command to spin up the flywheels to the specified RPMs
     *
     * @param leftRPM {@code int}: the target RPM of the left flywheel
     * (should be faster than the right if there is spin).
     * @param rightRPM {@code int}: the target RPM of the right flywheel
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted
     * to end
     */
    public Command primeCommand(int leftRPM, int rightRPM){
        return run(() -> {
            setShooterVelocity(leftRPM, rightRPM);
        });
    }

    /**
     * Command to spin up the flywheels to the specified RPMs
     *
     * @param leftRPM {@code IntSupplier}: a lambda that returns the target
     * RPM of the left flywheel (should be faster than the right if
     * there is spin).
     * @param rightRPM {@code IntSupplier}: a lambda that returns the target
     * RPM of the right flywheel.
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted
     * to end
     */
    public Command primeCommand(IntSupplier leftRPM, IntSupplier rightRPM){
        return run(() -> {
            setShooterVelocity(leftRPM.getAsInt(), rightRPM.getAsInt());
        });
    }

    /**
     * Command to spin up the flywheels to the RPMs stored in
     * the specified {@code RangeEntry}
     *
     * @param entry {@code RangeEntry}: the {@code RangeEntry} object containing 
     * the target flywheel speeds
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted
     * to end
     */
    public Command primeCommand(RangeTable.RangeEntry entry){
        return primeCommand(entry.leftFlywheelSpeed, entry.rightFlywheelSpeed);
    }

    /**
     * Command to spin up the flywheels to the RPMs in the supplied
     * {@code RangeEntry}
     *
     * @param entry {@code Supplier<RangeEntry>}: a lambda that returns a
     * {@code RangeEntry} object containing the target flywheel speeds.
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted
     * to end
     */
    public Command primeCommand(Supplier<RangeTable.RangeEntry> entry){
        return primeCommand(() -> entry.get().leftFlywheelSpeed, () -> entry.get().rightFlywheelSpeed);
    }

    /**
     * Command to run the note in the shooter (if there is one) into the flywheels.
     * There should be code elsewhere that blocks this from running if the flywheels
     * aren't spinning.
     *
     * @return the command
     *
     * @apiNote This command ends after {@link SHOOTER#SHOOT_SCORE_TIME} seconds
     */
    public Command fireCommand(){
        return run(() -> {
            feederMotor.setPercentOutput(SHOOTER.SHOOTING_FEEDER_POWER);
        }).withTimeout(SHOOTER.SHOOT_SCORE_TIME);
    }

    /**
     * Command to run the flywheels and feeders backwards to score in the amp. There
     * should be code elsewhere preventing this from running if the elevator isn't in
     * the amp position.
     *
     * @return the command
     *
     * @apiNote This command ends after {@link SHOOTER#AMP_SCORE_TIME} seconds
     */
    public Command ampScoreCommand(){
        return run(() -> {
            feederMotor.setVelocity(SHOOTER.AMP_FEEDER_SPEED);
            setFlywheelVelocity(SHOOTER.AMP_FLYWHEEL_SPEED);
        }).withTimeout(SHOOTER.AMP_SCORE_TIME);
    }

    /**
     * Command to stop the flywheels and feeder
     *
     * @return the command
     *
     * @apiNote This command runs instantly and
     * ends on the same frame
     */
    public Command stopCommand(){
        return runOnce(() -> {
            leftShooterMotor.setVelocity(0);
            leftTargetSpeed = 0;
            rightShooterMotor.setVelocity(0);
            rightTargetSpeed = 0;
            feederMotor.setVelocity(0);
        });
    }

    /**
     * Command to stop everything in preparation for auto
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command autonomousInitCommand(){
        return stopCommand();
    }


    /**
     * Command to run the no-note-yet part of the intake routine. This
     * should be run with the rest of the intake routine.
     *
     * @return the command
     *
     * @apiNote This command runs until {@link Shooter#isBottomBeamBreakTripped()}
     * returns {@code true}
     */
    public Command intakeNoContactCommand(){
        return run(() -> {
            feederMotor.setVelocity(SHOOTER.INTAKE_FEEDER_SPEED, 0);
            setFlywheelVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED);
        }).until(() -> isBottomBeamBreakTripped());
    }

    /**
     * Command to run the part of the intake routine that gets the note
     * from the of the feeder to the flywheels.
     *
     * @return the command
     *
     * @apiNote This command runs until {@link Shooter#isBottomBeamBreakTripped()}
     * returns {@code false}
     */
    public Command intakeWithContactCommand(){
        return run(() -> {
            feederMotor.setPercentOutput(SHOOTER.INTAKE_FEEDER_POWER);
            setFlywheelVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED);
        }).until(() -> !isBottomBeamBreakTripped());
    }

    /**
     * Command to adjust the note's position to get it in a good spot
     * for shooting.
     *
     * @return the command
     */
    public Command intakeAdjustNoteCommand(){
        return run(
            () -> {
                setFlywheelVelocity(SHOOTER.ALIGN_FLYWHEEL_SPEED);
                feederMotor.setVelocity(SHOOTER.ALIGN_FEEDER_SPEED, 1);
            }
        ).until(
            () -> isTopBeamBreakTripped() && feederMotor.getVelocity() < 0
        ).andThen(
            run(() -> {
                setFlywheelVelocity(SHOOTER.ALIGN_FLYWHEEL_SPEED);
                feederMotor.setVelocity(SHOOTER.ALIGN_FEEDER_SPEED, 1);
            }
        ).until(
            () -> !isTopBeamBreakTripped())
        );
    }

    /**
     * Command that runs the feeder and flywheels backwards. Usually
     * needs to be run with {@link Intake#outakeCommand()} as well.
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be
     * interrupted to end
     */
    public Command outakeCommand(){
        return run(() -> {
            setFlywheelVelocity(SHOOTER.OUTAKE_FLYWHEEL_SPEED);
            feederMotor.setVelocity(SHOOTER.OUTAKE_FEEDER_SPEED);
        });
    }

    /**
     * Command to run the passthrough routine. Usually needs to be
     * paired with the {@link Intake#intakeCommand()}.
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be
     * interrupted to end
     */
    public Command passThroughCommand(){
        return run(() -> {
            if(isBottomBeamBreakTripped()){
                feederMotor.setVelocity(1);
            }
            else{
                feederMotor.setVelocity(SHOOTER.INTAKE_FEEDER_SPEED, 0);
            }
            setFlywheelVelocity(5000);
        });
    }



    public void periodic() {
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/LeftTargetSpeed", leftTargetSpeed);
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/RightTargetSpeed", rightTargetSpeed);
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/RightShooterSpeed", rightShooterMotor.getVelocity());
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/LeftShooterSpeed", leftShooterMotor.getVelocity());
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/FeederSpeed", feederMotor.getVelocity());
        Logger.recordOutput(SHOOTER.LOG_PATH+"ReadyToShoot", readyToShoot());
        Logger.recordOutput(SHOOTER.LOG_PATH+"BeamBreaks/Bottom", bottomBeamBreak.get());
        Logger.recordOutput(SHOOTER.LOG_PATH+"BeamBreaks/Middle", middleBeamBreak.get());
        Logger.recordOutput(SHOOTER.LOG_PATH+"BeamBreaks/Top", topBeamBreak.get());
    }
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * @return whether the flywheels have spun up to within tolerance of their targets.
     */
    public boolean readyToShoot() {
        if (Math.abs(leftShooterMotor.getVelocity() - leftTargetSpeed) < SHOOTER.FHYWHEEL_SPEED_ACCEPTABLE_RANGE
                // && Math.abs(rightShooterMotor.getVelocity() - rightTargetSpeed) < SHOOTER.FHYWHEEL_SPEED_ACCEPTABLE_RANGE
                && leftTargetSpeed > 1000
                && rightTargetSpeed > 1000) {
            return true;
        }
        return false;
    }

    /**
     * Set the velocity of both flywheels. Runs {@link Shooter#setShooterVelocity(int, int)}
     * with the passed-in int.
     * @param both {@code int}: the speed in RPM to target for both flywheels
     */
    private void setFlywheelVelocity(int both){
        setShooterVelocity(both, both);
    }

    /**
     * Set the velocity of each flywheel.
     * @param left {@code int}: the velocity for the left flywheel
     * @param right {@code int}: the velocity for the right flywheel
     */
    private void setShooterVelocity(int left, int right){
        leftTargetSpeed = left;
        rightTargetSpeed = right;
        leftShooterMotor.setVelocity(leftTargetSpeed);
        rightShooterMotor.setVelocity(rightTargetSpeed);
    }

    /**
     * Returns {@code true} if the bottom beam-break sees a note.
     */
    private boolean isBottomBeamBreakTripped(){
        return !bottomBeamBreak.get(); //The beam break pulls low when triggered (notice exclamation point)
    }

    /**
     * Returns {@code true} if the top beam-break sees a note.
     */
    private boolean isTopBeamBreakTripped(){
        return !topBeamBreak.get(); //The beam break pulls low when triggered (notice exclamation point)
    }

    /**
     * Returns {@code true} if the middle beam-break sees a note.
     */
    public boolean isMiddleBeamBreakTripped(){
        return !middleBeamBreak.get(); //The beam break pulls low when triggered (notice exclamation point)
    }
}
