// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import frc.robot.helpers.*;

public class Shooter extends SubsystemBase {
    private SparkFlexControl leftShooterMotor;
    private SparkFlexControl rightShooterMotor;
    private SparkFlexControl feederMotor;

    private DigitalInput bottomBeamBreak;
    private DigitalInput topBeamBreak;
    private DigitalInput middleBeamBreak;

    int leftTargetSpeed = 0;
    int rightTargetSpeed = 0;

    public Shooter() {
        leftShooterMotor = new SparkFlexControl (CAN.TOP_SHOOTER_MOTOR_CAN_ID, false);
        leftShooterMotor.setPIDF(SHOOTER.LEFT_SHOOTER_MOTOR_kP,  SHOOTER.LEFT_SHOOTER_MOTOR_kI,  SHOOTER.LEFT_SHOOTER_MOTOR_kD,  SHOOTER.LEFT_SHOOTER_MOTOR_kF,  0);
        leftShooterMotor.motorControl.setIZone (SHOOTER.SHOOTER_MOTOR_IZONE);
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



    public Command shooterPrimeCommand(int leftRPM, int rightRPM){
        return new ShooterPrimeCommand(leftRPM, rightRPM).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
    public Command shooterPrimeCommand(IntSupplier leftRPM, IntSupplier rightRPM){
        return new ShooterPrimeCommand(leftRPM, rightRPM).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
    public Command shooterPrimeCommand(Supplier<RangeTable.RangeEntry> entry){
        return new ShooterPrimeCommand(entry).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
    public Command shooterPrimeCommand(RangeTable.RangeEntry entry){
        return new ShooterPrimeCommand(entry).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command fireCommand(){
        return run(() -> {
            feederMotor.setPercentOutput(SHOOTER.SHOOTING_FEEDER_POWER);
        }).withTimeout(SHOOTER.SHOOT_SCORE_TIME);
    }

    public Command ampScoreCommand(){
        return run(() -> {
            feederMotor.setVelocity(SHOOTER.AMP_FEEDER_SPEED);
            setShooterVelocity(SHOOTER.AMP_FLYWHEEL_SPEED);
        }).withTimeout(SHOOTER.AMP_SCORE_TIME);
    }

    public Command stopCommand(){
        return runOnce(() -> {
            leftShooterMotor.setVelocity(0);
            leftTargetSpeed = 0;
            rightShooterMotor.setVelocity(0);
            rightTargetSpeed = 0;
            feederMotor.setVelocity(0);
        });
    }



    public Command intakeNoContactCommand(){
        return run(() -> {
            feederMotor.setVelocity(SHOOTER.INTAKE_FEEDER_SPEED, 0); // Set PID to when note is disenganged
            setShooterVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED);
        }).until(() -> isBottomBeamBreakTripped());
    }
    public Command intakeWithContactCommand(){
        return run(() -> {
            feederMotor.setPercentOutput(SHOOTER.INTAKE_FEEDER_POWER); // Set PID to when note is disenganged
            setShooterVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED);
        }).until(() -> !isBottomBeamBreakTripped());
    }

    public Command intakeAdjustNoteCommand(){
        return run(() -> {
            setShooterVelocity(SHOOTER.ALIGN_FLYWHEEL_SPEED);
            feederMotor.setVelocity(SHOOTER.ALIGN_FEEDER_SPEED, 1);
        }).until(() -> isTopBeamBreakTripped() && feederMotor.getVelocity() < 0)

        .andThen(run(() -> {
            setShooterVelocity(SHOOTER.ALIGN_FLYWHEEL_SPEED);
            feederMotor.setVelocity(SHOOTER.ALIGN_FEEDER_SPEED, 1);
        }).until(() -> !isTopBeamBreakTripped()));
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
     * checks if flywheels are at target speed to shoot
     * and resets hasNote cuz that means we're gonna shoot
     * 
     * @apiNote You must call this method before shooting
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

    private void setShooterVelocity(int both){
        setShooterVelocity(both, both);
    }
    private void setShooterVelocity(int left, int right){
        leftTargetSpeed = left;
        rightTargetSpeed = right;
        leftShooterMotor.setVelocity(leftTargetSpeed);
        rightShooterMotor.setVelocity(rightTargetSpeed);
    }

    private boolean isBottomBeamBreakTripped(){
        return !bottomBeamBreak.get(); //The beam break pulls low when triggered (notice exclamation point)
    }

    private boolean isTopBeamBreakTripped(){
        return !topBeamBreak.get(); //The beam break pulls low when triggered (notice exclamation point)
    }

    private boolean isMiddleBeamBreakTripped(){
        return !middleBeamBreak.get(); //The beam break pulls low when triggered (notice exclamation point)
    }



    private class ShooterPrimeCommand extends Command{
        private IntSupplier leftRPM;
        private IntSupplier rightRPM;
        private boolean canEnd = true; //If one of the constructors with supplier inputs is called, this gets set to false and the command runs until interrupted
        public ShooterPrimeCommand(IntSupplier leftRPM, IntSupplier rightRPM){
            this.leftRPM = leftRPM;
            this.rightRPM = rightRPM;
            canEnd = false;
            addRequirements(Shooter.this); //Refers to the instance of Shooter that parents this instance of ShooterPrimeCommand.
        }
        public ShooterPrimeCommand(int leftRPM, int rightRPM){
            this(() -> leftRPM, () -> rightRPM);
            canEnd = true;
        }
        public ShooterPrimeCommand(Supplier<RangeTable.RangeEntry> entrySupplier){
            this(() -> entrySupplier.get().leftFlywheelSpeed, () -> entrySupplier.get().rightFlywheelSpeed);
            canEnd = false;
        }
        public ShooterPrimeCommand(RangeTable.RangeEntry entry){
            this(() -> entry.leftFlywheelSpeed, () -> entry.rightFlywheelSpeed);
            canEnd = true;
        }
        public void initialize(){}
        public void execute(){setShooterVelocity(leftRPM.getAsInt(), rightRPM.getAsInt());}
        public void end(boolean interrupted){
            if(interrupted){
                setShooterVelocity(0, 0);
            }
        }
        public boolean isFinished(){return readyToShoot() && canEnd;}
    }
}
