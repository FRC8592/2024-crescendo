// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.*;

import org.littletonrobotics.junction.Logger;
import frc.robot.helpers.*;
import frc.robot.subsystems.NewtonSubsystem;

public class Shooter extends NewtonSubsystem {
    private static Shooter instance = null;
    public static Shooter getInstance(){
        if(instance == null){
            throw new IllegalStateException("The Shooter subsystem must be instantiated before attempting to use it");
        }
        return instance;
    }
    public static Shooter instantiate(){
        if(instance != null){
            throw new IllegalStateException("The Shooter subsystem can't be instantiated twice");
        }
        instance = new Shooter();
        return instance;
    }

    public ShooterCommands commands = new ShooterCommands(this);

    private SparkFlexControl leftShooterMotor;
    private SparkFlexControl rightShooterMotor;
    private SparkFlexControl feederMotor;

    private DigitalInput bottomBeamBreak;
    private DigitalInput topBeamBreak;
    private DigitalInput middleBeamBreak;

    // Only used for logging
    private int leftTargetSpeed = 0;
    private int rightTargetSpeed = 0;
    private int feederTargetSpeed = 0;

    private Shooter() {
        leftShooterMotor = new SparkFlexControl(CAN.TOP_SHOOTER_MOTOR_CAN_ID, false);
        leftShooterMotor.setPIDF(SHOOTER.LEFT_SHOOTER_MOTOR_kP, SHOOTER.LEFT_SHOOTER_MOTOR_kI, SHOOTER.LEFT_SHOOTER_MOTOR_kD, SHOOTER.LEFT_SHOOTER_MOTOR_kF, 0);
        leftShooterMotor.motorControl.setIZone(SHOOTER.SHOOTER_MOTOR_IZONE);
        leftShooterMotor.setCurrentLimit (POWER.LEFT_SHOOTER_MOTOR_CURRENT_LIMIT, POWER.LEFT_SHOOTER_MOTOR_CURRENT_LIMIT);
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

    public void periodic() {
        Logger.recordOutput(SHOOTER.LOG_PATH+"Left/TargetSpeed", leftTargetSpeed);
        Logger.recordOutput(SHOOTER.LOG_PATH+"Left/ActualSpeed", leftShooterMotor.getVelocity());

        Logger.recordOutput(SHOOTER.LOG_PATH+"Right/TargetSpeed", rightTargetSpeed);
        Logger.recordOutput(SHOOTER.LOG_PATH+"Right/ActualSpeed", rightShooterMotor.getVelocity());

        Logger.recordOutput(SHOOTER.LOG_PATH+"Feeder/TargetSpeed", feederTargetSpeed);
        Logger.recordOutput(SHOOTER.LOG_PATH+"Feeder/ActualSpeed", feederMotor.getVelocity());

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
        if (Math.abs(leftShooterMotor.getVelocity() - leftTargetSpeed) < SHOOTER.FLYWHEEL_SPEED_ACCEPTABLE_RANGE
                // && Math.abs(rightShooterMotor.getVelocity() - rightTargetSpeed) < SHOOTER.FHYWHEEL_SPEED_ACCEPTABLE_RANGE
                && leftTargetSpeed > 1000
                && rightTargetSpeed > 1000) {
            return true;
        }
        return false;
    }

    /**
     * Set the velocity of both flywheels. Runs {@link Shooter#setShooterVelocity(int, int)}
     * with the passed-in {@code int}.
     *
     * @param both the speed in RPM to target for both flywheels
     */
    protected void setShooterVelocity(int both){
        setShooterVelocity(both, both);
    }

    /**
     * Set the velocity of each flywheel.
     *
     * @param left the velocity for the left flywheel
     * @param right the velocity for the right flywheel
     */
    protected void setShooterVelocity(int left, int right){
        leftTargetSpeed = left;
        rightTargetSpeed = right;
        leftShooterMotor.setVelocity(leftTargetSpeed);
        rightShooterMotor.setVelocity(rightTargetSpeed);
    }

    /**
     * @return the left motor's velocity in RPM
     */
    protected double getLeftVelocity(){
        return leftShooterMotor.getVelocity();
    }

    /**
     * @return the right motor's velocity in RPM
     */
    protected double getRightVelocity(){
        return rightShooterMotor.getVelocity();
    }

    /**
     * Run the feeder motor at a set power
     *
     * @param power the power to apply, (-1 to 1)
     */
    protected void setFeederPower(double power){
        feederTargetSpeed = -1;
        feederMotor.setPercentOutput(power);
    }

    /**
     * Run the feeder motor at a set velocity using PID slot 0
     *
     * @param velocity the velocity to run the motor at in RPM
     */
    protected void setFeederVelocity(int velocity){
        feederTargetSpeed = velocity;
        feederMotor.setVelocity(velocity);
    }

    /**
     * Run the feeder motor at a set velocity using specific PID constants
     *
     * @param velocity the velocity to run the motor at in RPM
     * @param pidSlot the PID slot to use
     */
    protected void setFeederVelocity(int velocity, int pidSlot){
        feederTargetSpeed = velocity;
        feederMotor.setVelocity(velocity, pidSlot);
    }

    /**
     * Stop all shooter motors
     */
    protected void stop(){
        setFeederVelocity(0);
        setShooterVelocity(0);
    }

    /**
     * @return the feeder motor's current measured velocity
     */
    protected double getFeederVelocity(){
        return feederMotor.getVelocity();
    }

    /**
     * Returns {@code true} if the bottom beam-break sees a note.
     */
    protected boolean isBottomBeamBreakTripped(){
        return !bottomBeamBreak.get(); //The beam break pulls low when triggered (notice exclamation point)
    }

    /**
     * Returns {@code true} if the top beam-break sees a note.
     */
    protected boolean isTopBeamBreakTripped(){
        return !topBeamBreak.get(); //The beam break pulls low when triggered (notice exclamation point)
    }

    /**
     * Returns {@code true} if the middle beam-break sees a note.
     */
    public boolean isMiddleBeamBreakTripped(){
        return !middleBeamBreak.get(); //The beam break pulls low when triggered (notice exclamation point)
    }
}
