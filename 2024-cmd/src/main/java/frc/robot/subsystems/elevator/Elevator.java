// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.*;
import frc.robot.Constants.*;

public class Elevator extends SubsystemBase {
    public ElevatorCommands commands = new ElevatorCommands(this);

    // Small enum for conveniently referencing elevator positions
    public enum Positions{
        // The next three lines do the equivalent of the code
        // public Positions NAME = new Positions(pivotAngle, extensionLength);
        // (creating new instances of Positions using the constructor below).
        STOWED(ELEVATOR.PIVOT_ANGLE_STOWED, ELEVATOR.EXTENSION_METERS_STOWED),
        AMP(ELEVATOR.PIVOT_ANGLE_AMP, ELEVATOR.EXTENSION_METERS_AMP),
        CLIMB(ELEVATOR.PIVOT_ANGLE_CLIMB, ELEVATOR.EXTENSION_METERS_CLIMB),;

        // These are enum variables, which act just like variables of any class
        public double pivot;
        public double extension;

        // Enum constructor
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
     * Check whether the elevator is within tolerance of the specified position
     *
     * @param pivot the angle to check for the pivot being within tolerance of
     * @param extension the length to check for the extension being within
     * tolerance of
     *
     * @return whether both the pivot and extension are within tolerance of the
     * passed-in values
     *
     * @apiNote This is a positional check only. If the elevator is at the
     * specified position but is moving towards a different target, this method
     * will still return {@code true}. To require that the elevator be targeting
     * the same position, logical-AND this function with
     * {@link Elevator#isTargeting(double, double)}
     */
    public boolean isAtPosition(double pivot, double extension){
        return (
            Math.abs(getPivotAngle() - pivot) < ELEVATOR.ANGLE_TOLERANCE
            && Math.abs(getExtensionLength() - extension) < ELEVATOR.LENGTH_TOLERANCE
        );
    }

    /**
     * Check whether the elevator is within tolerance of the specified position
     *
     * @param position the position to check for the elevator being within
     * tolerance of
     *
     * @return whether both the pivot and extension are within tolerance of the
     * passed-in {@code Position}
     *
     * @apiNote This is a positional check only. If the elevator is at the
     * specified position but is moving towards a different target, this method
     * will still return {@code true}. To require that the elevator be targeting
     * the same position, logical-AND this function with
     * {@link Elevator#isTargeting(Positions)}
     */
    public boolean isAtPosition(Positions position){
        return (
            Math.abs(getPivotAngle() - position.pivot) < ELEVATOR.ANGLE_TOLERANCE
            && Math.abs(getExtensionLength() - position.extension) < ELEVATOR.LENGTH_TOLERANCE
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
     * Set the target pivot angle used by {@link Elevator#runElevator()}
     * @param pivotAngle the angle setpoint in degrees
     */
    protected void setTargetPivot(double pivotAngle){
        this.targetPivot = pivotAngle;
    }

    /**
     * Set the target extension length used by {@link Elevator#runElevator()}
     * @param extensionLength the extension setpoint in meters
     */
    protected void setTargetExtension(double extensionLength){
        this.targetExtension = extensionLength;
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
     * Check the elevator's targets
     *
     * @param pivot the pivot angle to check that the elevator is targeting
     * @param extension the extension length to check that the elevator is
     * targeting
     *
     * @return whether the elevator's target angle and extension exactly match
     * the passed-in values.
     */
    public boolean isTargeting(double pivot, double extension){
        return getTargetPivot() == pivot && getTargetExtension() == extension;
    }

    /**
     * Check the elevator's targets
     *
     * @param position the position to check that the elevator is targeting
     *
     * @return whether the elevator's target angle and extension exactly
     * match the values in the {@code Position} object.
     */
    public boolean isTargeting(Positions position){
        return isTargeting(position.pivot, position.extension);
    }

    /**
     * Method to run the elevator with safety features. DO NOT COMMAND THE ELEVATOR
     * MOTORS OTHER THAN THROUGH THIS METHOD unless you are stopping them.
     */
    protected void runElevator(){
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

    /**
     * Stop the elevator motors. Uses a positional setpoint to stop, meaning
     * the elevator can reverse (as needed) to reach where it was when commanded
     * to stop.
     */
    protected void freezeElevator(){
        pivotMotor.setPositionSmartMotion(pivotMotor.getPosition());
        extensionMotor.setPositionSmartMotion(extensionMotor.getPosition());
    }
}