// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.*;
import frc.robot.Constants.*;

public class Elevator extends SubsystemBase {
    private SparkFlexControl extensionMotor;
    public SparkFlexControl pivotMotor;
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

    private class SetElevatorPositionCommand extends Command{
        public SetElevatorPositionCommand(double pivotDegrees, double extensionMeters){
            targetExtension = extensionMeters;
            targetPivot = pivotDegrees;
            addRequirements(Elevator.this);
        }
        public void initialize(){}
        public void execute(){
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
        public void end(boolean interrupted){
            //If we interrupt, we no longer have the above logic protecting the elevator, so we freeze the elevator
            if(interrupted){
                extensionMotor.setPositionSmartMotion(extensionMotor.getPosition());
                pivotMotor.setPositionSmartMotion(pivotMotor.getPosition());
            }
        }
        public boolean isFinished(){return isAtTargetPosition();}
    }

    public Command setElevatorPositionCommand(double pivotDegrees, double extensionMeters) {
        return new SetElevatorPositionCommand(pivotDegrees, extensionMeters);
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
}
