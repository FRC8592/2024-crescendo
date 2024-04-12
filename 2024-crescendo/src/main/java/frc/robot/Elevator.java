package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

public class Elevator {
    private SparkFlexControl extensionMotor;
    public SparkFlexControl pivotMotor;
    private SparkFlexControl pivotFollowMotor;
    private AbsoluteEncoder pivotEncoder;

    private double desiredExtension;
    private double desiredPivot;
    private double targetExtension;
    private double targetPivot;

    public Elevator(){
        extensionMotor = new SparkFlexControl(CAN.ELEVATOR_MOTOR_CAN_ID, false);
        pivotMotor = new SparkFlexControl(CAN.PIVOT_MOTOR_CAN_ID, false);
        pivotFollowMotor = new SparkFlexControl(CAN.PIVOT_FOLLOW_MOTOR_CAN_ID, false);

        pivotMotor.setPIDF(ELEVATOR.PIVOT_kP, ELEVATOR.PIVOT_kI, ELEVATOR.PIVOT_kD, ELEVATOR.PIVOT_kFF, 0);
        pivotFollowMotor.setPIDF(ELEVATOR.PIVOT_kP, ELEVATOR.PIVOT_kI, ELEVATOR.PIVOT_kD, ELEVATOR.PIVOT_kFF, 0);

        extensionMotor.setPIDF(ELEVATOR.EXTENSION_kP, ELEVATOR.EXTENSION_kI, ELEVATOR.EXTENSION_kD, ELEVATOR.EXTENSION_kFF, 0);

        pivotMotor.setInverted();
        pivotMotor.motorControl.setIZone(ELEVATOR.PIVOT_IZONE * ELEVATOR.PIVOT_GEAR_RATIO); // pivot degrees

        pivotMotor.setMaxVelocity(6500, 0);
        pivotFollowMotor.setMaxVelocity(6500, 0);

        extensionMotor.setMaxVelocity(5000, 0); // TODO: Why is this so slow?????

        pivotMotor.setMaxAcceleration(7000, 0);
        pivotFollowMotor.setMaxAcceleration(7000, 0);

        extensionMotor.setMaxAcceleration(10000, 0);
        
        pivotMotor.motorControl.setReference(0,ControlType.kVoltage);
    }

    
    /**
     * Called in robot periodic - checks variables and updates motors
     */
    public void update(){
        double actualPivot = getPivotAngle();
        double actualExtension = getExtensionLength();

        // Keep the target pivot angle within bounds
        targetPivot = Math.max(Math.min(desiredPivot, ELEVATOR.PIVOT_ANGLE_MAX), ELEVATOR.PIVOT_ANGLE_MIN);

        // Keep the target extension length within bounds
        targetExtension = Math.max(Math.min(desiredExtension, ELEVATOR.EXTENSION_METERS_MAX), ELEVATOR.EXTENSION_METERS_MIN);

        // If we're close to the threshold pivot angle (30°) and the extension isn't
        // fully retracted, keep the pivot at a safe height but allow the extension to
        // do whatever it wants.
        if (actualPivot < ELEVATOR.EXTENSION_FORCE_RETRACT_THRESHOLD - ELEVATOR.RETRACT_THRESHOLD_TOLERANCE){
            targetExtension = actualExtension;
        }

        // If we're below "close to the threshold," and still extended, stop the
        // extension and bring the pivot back up.
        if (actualExtension > ELEVATOR.EXTENSION_FULLY_RETRACTED && targetPivot < ELEVATOR.EXTENSION_FORCE_RETRACT_THRESHOLD) {
            targetPivot = ELEVATOR.EXTENSION_FORCE_RETRACT_THRESHOLD;
        }

        double rotations = targetExtension / ELEVATOR.ELEVATOR_GEAR_RATIO;
        extensionMotor.setPositionSmartMotion(rotations);
        double pivotRotations = (ELEVATOR.PIVOT_GEAR_RATIO * targetPivot) / 360;
        pivotMotor.setPositionSmartMotion(pivotRotations);
        pivotFollowMotor.setPositionSmartMotion(pivotRotations);
    }

    //-------ELEVATOR CODE-------//

    /**
     * sets the speed of
     * @param speed
     */
    public void percentOutputExtension(double speed){
        extensionMotor.setPercentOutput(speed);
    }

    public void setExtensionLengthCustom(double position){
        desiredExtension = position;
    }

    /**
     * gets length of elevator in meters
     * @return
     */
    public double getExtensionLength() {
        return (extensionMotor.getPosition()*ELEVATOR.ELEVATOR_GEAR_RATIO);
    }

    //-------PIVOT CODE-------//

    /**
     * sets the speed of
     * @param speed
     */
    public void percentOutputPivot(double speed){
        pivotMotor.setPercentOutput(speed);
        pivotFollowMotor.setPercentOutput(speed);
    }

    public void setPivotAngleCustom(double angle) {
        desiredPivot = angle;
    }

    /**
     * gets position of pivot
     * @return
     */
    public double getPivotAngle() {
        double ticksConverted = (pivotMotor.getTicks()*CONVERSIONS.TICKS_TO_ANGLE_DEGREES_SPARKFLEX)/ELEVATOR.PIVOT_GEAR_RATIO;
        return ticksConverted;
    }

    public boolean isTargetAngle() {
        return Math.abs(getPivotAngle() - desiredPivot) < ELEVATOR.ANGLE_TOLERANCE;
    }

    public boolean isTargetLength() {
        return Math.abs(getExtensionLength() - desiredExtension) < ELEVATOR.LENGTH_TOLERANCE;
    }

    //-------COMBINED CODE-------//

    /**
     * stows elevator and pivot 
     */
    public void stow() {
        desiredExtension = ELEVATOR.EXTENSION_METERS_STOWED;
        desiredPivot = ELEVATOR.PIVOT_ANGLE_STOWED;
    }

    public void ampPosition() {
        desiredExtension = ELEVATOR.EXTENSION_METERS_AMP;
        desiredPivot = ELEVATOR.PIVOT_ANGLE_AMP;
    }

    public void climbPosition(){
        desiredExtension = ELEVATOR.EXTENSION_METERS_CLIMB;
        desiredPivot = ELEVATOR.PIVOT_ANGLE_CLIMB;
    }

    /**
     * zeros the pivot encoder by setting the offset to the current position
     */
    public void zero(){
        pivotEncoder.setZeroOffset(pivotEncoder.getPosition());
    }
    
    /**
     * resets the encoders for pivot motor, pivot follow motor, extension motor
    */
    public void resetEncoders() {
        pivotMotor.motorEncoder.setPosition(0);
        pivotFollowMotor.motorEncoder.setPosition(0);
        extensionMotor.motorEncoder.setPosition(0);
    }

    public void extend() {
        desiredExtension += ELEVATOR.MANUAL_EXTENSION_SPEED;
        desiredExtension = Math.min(desiredExtension, ELEVATOR.EXTENSION_METERS_MAX);
    }

    public void retract() {
        desiredExtension -= ELEVATOR.MANUAL_EXTENSION_SPEED;
        desiredExtension = Math.max(desiredExtension, 0);
    }

    public void setElevatorPosition(double angle, double length){
        setExtensionLengthCustom(length);
        setPivotAngleCustom(angle);
    }

    public boolean isAtTargetPosition(){
        return isTargetAngle() && isTargetLength();
    }
}
