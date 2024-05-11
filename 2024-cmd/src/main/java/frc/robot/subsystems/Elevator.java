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
        }).until(() -> isAtTargetPosition());
    }

    public Command setUpdatingPositionCommand(DoubleSupplier pivotDegrees, DoubleSupplier extensionMeters){
        return run(() -> {
            targetPivot = pivotDegrees.getAsDouble();
            targetExtension = extensionMeters.getAsDouble();
            runElevator();
        });
    }

    public Command setMalleablePositionCommand(double pivotDegrees, double extensionMeters){
        targetPivot = pivotDegrees;
        targetExtension = extensionMeters;
        return run(() -> {
            runElevator();
        });
    }
    public Command incrementElevatorPositionCommand(double pivotDegrees, double extensionMeters){
        return runOnce(() -> {
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

    private boolean isAtTargetPosition(){
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



    // private class SetElevatorPositionCommand extends Command{
    //     private DoubleSupplier pivotDegreesSupplier;
    //     private DoubleSupplier extensionMetersSupplier;
    //     private boolean canEnd;
    //     public SetElevatorPositionCommand(DoubleSupplier pivotDegrees, DoubleSupplier extensionMeters, boolean ends){
    //         pivotDegreesSupplier = pivotDegrees;
    //         extensionMetersSupplier = extensionMeters;
    //         addRequirements(Elevator.this);
    //         canEnd = ends;
    //     }
    //     public SetElevatorPositionCommand(double pivotDegrees, double extensionMeters, boolean ends){
    //         this(() -> pivotDegrees, () -> extensionMeters, ends);
    //     }
    //     public SetElevatorPositionCommand(Supplier<RangeTable.RangeEntry> entrySupplier, boolean ends){
    //         this(() -> entrySupplier.get().pivotAngle, () -> entrySupplier.get().elevatorHeight, ends);
    //     }
    //     public SetElevatorPositionCommand(RangeTable.RangeEntry entry, boolean ends){
    //         this(() -> entry.pivotAngle, () -> entry.elevatorHeight, ends);
    //     }
    //     public void initialize(){}
    //     public void execute(){
    //         targetExtension = extensionMetersSupplier.getAsDouble();
    //         targetPivot = pivotDegreesSupplier.getAsDouble();
    //         runElevator();
    //     }
    //     public void end(boolean interrupted){
    //         //If we interrupt, we no longer have the above logic protecting the elevator, so we freeze the elevator
    //         if(interrupted){
    //             extensionMotor.setPositionSmartMotion(extensionMotor.getPosition());
    //             pivotMotor.setPositionSmartMotion(pivotMotor.getPosition());
    //         }
    //     }
    //     public boolean isFinished(){return isAtTargetPosition() && canEnd;}
    // }
}