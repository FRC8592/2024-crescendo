// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.helpers.*;
import frc.robot.Constants.*;

public class Elevator extends SubsystemBase {
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

    // public Command setElevatorPositionCommand(double pivotDegrees, double extensionMeters, boolean ends) {
    //     return new SetElevatorPositionCommand(pivotDegrees, extensionMeters, ends);
    // }
    // public Command setElevatorPositionCommand(DoubleSupplier pivotDegrees, DoubleSupplier extensionMeters, boolean ends) {
    //     return new SetElevatorPositionCommand(pivotDegrees, extensionMeters, ends);
    // }
    // public Command setElevatorPositionCommand(Supplier<RangeTable.RangeEntry> entrySupplier, boolean ends) {
    //     return new SetElevatorPositionCommand(entrySupplier, ends);
    // }
    // public Command setElevatorPositionCommand(RangeTable.RangeEntry entry, boolean ends) {
    //     return new SetElevatorPositionCommand(entry, ends);
    // }

    public Command setStaticPositionCommand(double pivotDegrees, double extensionMeters){
        return run(() -> {
            targetPivot = pivotDegrees;
            targetExtension = extensionMeters;
            runElevator();
        }).until(() -> isAtTargetPosition())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command setUpdatingPositionCommand(DoubleSupplier pivotDegrees, DoubleSupplier extensionMeters){
        return run(() -> {
            targetPivot = pivotDegrees.getAsDouble();
            targetExtension = extensionMeters.getAsDouble();
            runElevator();
        })
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command setMalleablePositionCommand(double pivotDegrees, double extensionMeters){
        targetPivot = pivotDegrees;
        targetExtension = extensionMeters;
        return run(() -> {
            runElevator();
        })
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
    public Command incrementElevatorPositionCommand(double pivotDegrees, double extensionMeters){
        return run(() -> {
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
        return isAtTargetPosition()
                && targetExtension == ELEVATOR.EXTENSION_METERS_AMP
                && targetPivot == ELEVATOR.PIVOT_ANGLE_AMP;
    }

    public boolean isClimb(){
        return isAtTargetPosition()
                && targetExtension == ELEVATOR.EXTENSION_METERS_CLIMB
                && targetPivot == ELEVATOR.PIVOT_ANGLE_CLIMB;
    }

    public boolean isStowed(){
        return isAtTargetPosition()
                && targetExtension == ELEVATOR.EXTENSION_METERS_STOWED
                && targetPivot == ELEVATOR.PIVOT_ANGLE_STOWED;
    }

    private double getExtensionLength() {
        return (extensionMotor.getPosition()*ELEVATOR.ELEVATOR_GEAR_RATIO);
    }

    private double getPivotAngle() {
        double ticksConverted = (pivotMotor.getTicks()*CONVERSIONS.TICKS_TO_ANGLE_DEGREES_SPARKFLEX)/ELEVATOR.PIVOT_GEAR_RATIO;
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

        targetPivot = Math.max(Math.min(targetPivot, ELEVATOR.PIVOT_ANGLE_MAX), ELEVATOR.PIVOT_ANGLE_MIN);

        targetExtension = Math.max(Math.min(targetExtension, ELEVATOR.EXTENSION_METERS_MAX), ELEVATOR.EXTENSION_METERS_MIN);

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