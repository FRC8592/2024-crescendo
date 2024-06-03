// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.helpers.*;
import frc.robot.Constants.*;

public class Elevator extends NewtonSubsystem {

    // Small enum for conveniently referencing elevator positions
    public enum Positions{
        STOWED(ELEVATOR.PIVOT_ANGLE_STOWED, ELEVATOR.EXTENSION_METERS_STOWED),
        AMP(ELEVATOR.PIVOT_ANGLE_AMP, ELEVATOR.EXTENSION_METERS_AMP),
        CLIMB(ELEVATOR.PIVOT_ANGLE_CLIMB, ELEVATOR.EXTENSION_METERS_CLIMB),;

        public double pivot;
        public double extension;
        Positions(double pivot, double extension){
            this.pivot = pivot;
            this.extension = extension;
        }
    }

    private SparkFlexControl extensionMotor;
    private SparkFlexControl pivotMotor;
    private SparkFlexControl pivotFollowMotor;

    // Unlike the similarly named variables in Shooter, these are used for
    // subsystem control and aren't just logging
    private double targetExtension;
    private double targetPivot;

    public Elevator(){
        extensionMotor = new SparkFlexControl(CAN.ELEVATOR_MOTOR_CAN_ID, false);
        extensionMotor.setPIDF(ELEVATOR.EXTENSION_kP, ELEVATOR.EXTENSION_kI, ELEVATOR.EXTENSION_kD, ELEVATOR.EXTENSION_kFF, 0);
        extensionMotor.setMaxVelocity(5000, 0);
        extensionMotor.setMaxAcceleration(10000, 0);

        pivotMotor = new SparkFlexControl(CAN.PIVOT_MOTOR_CAN_ID, false);
        pivotMotor.setPIDF(ELEVATOR.PIVOT_kP, ELEVATOR.PIVOT_kI, ELEVATOR.PIVOT_kD, ELEVATOR.PIVOT_kFF, 0);
        pivotMotor.setMaxVelocity(6500, 0);
        pivotMotor.setMaxAcceleration(7000, 0); //Going higher than this caused slamming
        pivotMotor.motorControl.setIZone(ELEVATOR.PIVOT_IZONE * ELEVATOR.PIVOT_GEAR_RATIO);
        pivotMotor.motorControl.setReference(0, ControlType.kVoltage);
        pivotMotor.setInverted();

        pivotFollowMotor = new SparkFlexControl(CAN.PIVOT_FOLLOW_MOTOR_CAN_ID, false);
        pivotFollowMotor.setPIDF(ELEVATOR.PIVOT_kP, ELEVATOR.PIVOT_kI, ELEVATOR.PIVOT_kD, ELEVATOR.PIVOT_kFF, 0);
        pivotFollowMotor.setMaxVelocity(6500, 0);
        pivotFollowMotor.setMaxAcceleration(7000, 0);
    }

    /**
     * Command to put the elevator at the given setpoint. Ends when the elevator is at the target.
     *
     * @param pivotDegrees {@code double}: the pivot target in degrees
     * @param extensionMeters {@code double}: the extension target in meters
     *
     * @return the command
     *
     * @apiNote This command runs until {@link Elevator#isAtTargetPosition()} returns {@code true}
     */
    public Command setStaticPositionCommand(double pivotDegrees, double extensionMeters){
        return run(() -> {
            targetPivot = pivotDegrees;
            targetExtension = extensionMeters;
            runElevator();
        }).until(() -> isAtTargetPosition());
    }

    /**
     * Command to put the elevator at the given position. Ends when the elevator is at the target.
     *
     * @param position {@code Positions}: the position to go to
     *
     * @return the command
     *
     * @apiNote This command runs until {@link Elevator#isAtTargetPosition()} returns {@code true}
     */
    public Command setStaticPositionCommand(Positions position){
        return run(() -> {
            targetPivot = position.pivot;
            targetExtension = position.extension;
            runElevator();
        }).until(() -> isAtTargetPosition());
    }

    /**
     * Command to contantly drive the elevator towards a setpoint that can change as desired.
     * This command never ends on its own.
     *
     * @param pivotDegrees {@code DoubleSupplier}: a lambda that returns where the pivot should be
     * @param extensionMeters {@code DoubleSupplier}: a lambda that returns how far the extension
     * should go
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command setUpdatingPositionCommand(DoubleSupplier pivotDegrees, DoubleSupplier extensionMeters){
        return run(() -> {
            targetPivot = pivotDegrees.getAsDouble();
            targetExtension = extensionMeters.getAsDouble();
            runElevator();
        });
    }

    /**
     * The same as {@link Elevator#setStaticPositionCommand(double, double)}, but allows the setpoint
     * to be edited  on the fly and won't end on its own. This is meant to be used together with 
     * {@link Elevator#incrementElevatorPositionCommand(double, double)}.
     *
     * @param pivotDegrees {@code double}: the pivot setpoint to start with
     * @param extensionMeters {@code double}: the extension setpoint to start with
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command setMalleablePositionCommand(double pivotDegrees, double extensionMeters){
        targetPivot = pivotDegrees;
        targetExtension = extensionMeters;
        return run(() -> {
            runElevator();
        });
    }

    /**
     * The same as {@link Elevator#setStaticPositionCommand(Position)}, but allows the setpoint
     * to be edited  on the fly and won't end on its own. This is meant to be used together with 
     * {@link Elevator#incrementElevatorPositionCommand(double, double)}.
     *
     * @param position {@code Positions}: the position to go to
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command setMalleablePositionCommand(Positions position){
        targetPivot = position.pivot;
        targetExtension = position.extension;
        return run(() -> {
            runElevator();
        });
    }

    /**
     * If a {@link Elevator#setMalleablePositionCommand(double, double)} command is running, change
     * the pivot and extension setpoints for the elevator to target. Otherwise, does nothing functional.
     *
     * @param pivotDegrees {@code double}: the amount to change the pivot target by each frame
     * @param extensionMeters {@code double}: the amount to change the extension setpoint by each frame
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     * @apiNote This command does NOT require the Elevator subsystem
     */
    public Command incrementElevatorPositionCommand(double pivotDegrees, double extensionMeters){
        return Commands.run(() -> {
            this.targetPivot += pivotDegrees;
            this.targetExtension += extensionMeters;
        });
    }

    public void periodic() {
        Logger.recordOutput(ELEVATOR.LOG_PATH+"Pivot/Target", targetPivot);
        Logger.recordOutput(ELEVATOR.LOG_PATH+"Pivot/Read", getPivotAngle());
        Logger.recordOutput(ELEVATOR.LOG_PATH+"Extension/Target", targetExtension);
        Logger.recordOutput(ELEVATOR.LOG_PATH+"Extension/Read", getExtensionLength());
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * Return whether the elevator is within tolerance of the amp position
     */
    public boolean isAmp(){
        return (
            isAtTargetPosition()
            && targetExtension == ELEVATOR.EXTENSION_METERS_AMP
            && targetPivot == ELEVATOR.PIVOT_ANGLE_AMP
        );
    }

    /**
     * Return whether the elevator is within tolerance of the climb position
     */
    public boolean isClimb(){
        return (
            isAtTargetPosition()
            && targetExtension == ELEVATOR.EXTENSION_METERS_CLIMB
            && targetPivot == ELEVATOR.PIVOT_ANGLE_CLIMB
        );
    }

    /**
     * Return whether the elevator is within tolerance of the stowed position
     */
    public boolean isStowed(){
        return (
            isAtTargetPosition()
            && targetExtension == ELEVATOR.EXTENSION_METERS_STOWED
            && targetPivot == ELEVATOR.PIVOT_ANGLE_STOWED
        );
    }

    /**
     * Return the elevator's target extension length
     */
    public double getTargetExtension(){
        return this.targetExtension;
    }

    /**
     * Return the elevator's target pivot angle
     */
    public double getTargetPivot(){
        return this.targetPivot;
    }

    /**
     * Return the elevator's measured extension length
     */
    private double getExtensionLength() {
        return (extensionMotor.getPosition()*ELEVATOR.ELEVATOR_GEAR_RATIO);
    }

    /**
     * Return the elevator's measured pivot angle
     */
    private double getPivotAngle() {
        double ticksConverted = (
            (pivotMotor.getTicks()*CONVERSIONS.TICKS_TO_ANGLE_DEGREES_SPARKFLEX)
            /ELEVATOR.PIVOT_GEAR_RATIO
        );
        return ticksConverted;
    }

    /**
     * Return whether the elevator is at its target angle
     */
    private boolean isTargetAngle() {
        return Math.abs(getPivotAngle() - targetPivot) < ELEVATOR.ANGLE_TOLERANCE;
    }

    /**
     * Return whether the elevator is at its target length
     */
    private boolean isTargetLength() {
        return Math.abs(getExtensionLength() - targetExtension) < ELEVATOR.LENGTH_TOLERANCE;
    }

    /**
     * Return whether the elevator is at its target angle and length
     */
    public boolean isAtTargetPosition(){
        return isTargetAngle() && isTargetLength();
    }

    /**
     * Method to run the elevator with safety features. DO NOT COMMAND THE ELEVATOR
     * MOTORS OTHER THAN THROUGH THIS METHOD unless you are stopping them.
     */
    private void runElevator(){
        double actualPivot = getPivotAngle();
        double actualExtension = getExtensionLength();

        // If the target pivot angle is above maximum or below minimum,
        // force it back to a value within its physical capabilities
        targetPivot = Math.max(
            Math.min(
                targetPivot,
                ELEVATOR.PIVOT_ANGLE_MAX
            ),
            ELEVATOR.PIVOT_ANGLE_MIN
        );

        // If the target extension length is above maximum or below
        // minimum, force it back to a value within its physical
        // capabilities
        targetExtension = Math.max(
            Math.min(
                targetExtension,
                ELEVATOR.EXTENSION_METERS_MAX
            ),
            ELEVATOR.EXTENSION_METERS_MIN
        );

        // If the read pivot angle is outside of tolerance of the lowest the pivot can go with the extension
        // out, stop the extension (used to keep the extension in when going up from stowed)
        if (actualPivot < ELEVATOR.EXTENSION_FORCE_RETRACT_THRESHOLD - ELEVATOR.RETRACT_THRESHOLD_TOLERANCE){
            targetExtension = actualExtension;
        }

        // If the extension is out and the pivot is lower than it can go with the extension out, force the pivot
        // back up
        if (actualExtension > ELEVATOR.EXTENSION_FULLY_RETRACTED && targetPivot < ELEVATOR.EXTENSION_FORCE_RETRACT_THRESHOLD) {
            targetPivot = ELEVATOR.EXTENSION_FORCE_RETRACT_THRESHOLD;
        }

        // NOTE: the above two if-statements combined mean that the extension can't extend itself when the hooks
        // on it could hit something, and if it gets into the danger zone while extended (from the pivot going
        // down before the extension is fully in), the pivot will stay at a safe height until the extension is
        // fully retracted.

        double extensionRotations = targetExtension / ELEVATOR.ELEVATOR_GEAR_RATIO;
        extensionMotor.setPositionSmartMotion(extensionRotations);

        double pivotRotations = (ELEVATOR.PIVOT_GEAR_RATIO * targetPivot) / 360;
        pivotMotor.setPositionSmartMotion(pivotRotations);
        pivotFollowMotor.setPositionSmartMotion(pivotRotations);
    }
}