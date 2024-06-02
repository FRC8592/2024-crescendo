// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.*;
import frc.robot.Constants.*;

public class Elevator extends NewtonSubsystem {
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
        pivotMotor.setMaxAcceleration(7000, 0);
        pivotMotor.motorControl.setIZone(ELEVATOR.PIVOT_IZONE * ELEVATOR.PIVOT_GEAR_RATIO); // pivot degrees
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
     * @param pivotDegrees {@code double}: the amount to change the pivot target by
     * @param extensionMeters {@code double}: the amount to change the extension setpoint by
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

    public boolean isAmp(){
        return (
            isAtTargetPosition()
            && targetExtension == ELEVATOR.EXTENSION_METERS_AMP
            && targetPivot == ELEVATOR.PIVOT_ANGLE_AMP
        );
    }

    public boolean isClimb(){
        return (
            isAtTargetPosition()
            && targetExtension == ELEVATOR.EXTENSION_METERS_CLIMB
            && targetPivot == ELEVATOR.PIVOT_ANGLE_CLIMB
        );
    }

    public boolean isStowed(){
        return (
            isAtTargetPosition()
            && targetExtension == ELEVATOR.EXTENSION_METERS_STOWED
            && targetPivot == ELEVATOR.PIVOT_ANGLE_STOWED
        );
    }

    private double getExtensionLength() {
        return (extensionMotor.getPosition()*ELEVATOR.ELEVATOR_GEAR_RATIO);
    }

    private double getPivotAngle() {
        double ticksConverted = (
                (pivotMotor.getTicks()*CONVERSIONS.TICKS_TO_ANGLE_DEGREES_SPARKFLEX)
                /ELEVATOR.PIVOT_GEAR_RATIO
        );
        return ticksConverted;
    }

    private boolean isTargetAngle() {
        return Math.abs(getPivotAngle() - targetPivot) < ELEVATOR.ANGLE_TOLERANCE;
    }

    private boolean isTargetLength() {
        return Math.abs(getExtensionLength() - targetExtension) < ELEVATOR.LENGTH_TOLERANCE;
    }

    public boolean isAtTargetPosition(){
        return isTargetAngle() && isTargetLength();
    }

    private void runElevator(){
        double actualPivot = getPivotAngle();
        double actualExtension = getExtensionLength();

        targetPivot = Math.max(
            Math.min(
                targetPivot,
                ELEVATOR.PIVOT_ANGLE_MAX
            ),
            ELEVATOR.PIVOT_ANGLE_MIN
        );

        targetExtension = Math.max(
            Math.min(
                targetExtension,
                ELEVATOR.EXTENSION_METERS_MAX
            ),
            ELEVATOR.EXTENSION_METERS_MIN
        );

        if (actualPivot < ELEVATOR.EXTENSION_FORCE_RETRACT_THRESHOLD - ELEVATOR.RETRACT_THRESHOLD_TOLERANCE){
            targetExtension = actualExtension;
        }

        if (actualExtension > ELEVATOR.EXTENSION_FULLY_RETRACTED && targetPivot < ELEVATOR.EXTENSION_FORCE_RETRACT_THRESHOLD) {
            targetPivot = ELEVATOR.EXTENSION_FORCE_RETRACT_THRESHOLD;
        }

        double extensionRotations = targetExtension / ELEVATOR.ELEVATOR_GEAR_RATIO;
        extensionMotor.setPositionSmartMotion(extensionRotations);

        double pivotRotations = (ELEVATOR.PIVOT_GEAR_RATIO * targetPivot) / 360;
        pivotMotor.setPositionSmartMotion(pivotRotations);
        pivotFollowMotor.setPositionSmartMotion(pivotRotations);
    }
}