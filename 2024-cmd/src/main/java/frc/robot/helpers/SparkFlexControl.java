package frc.robot.helpers;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class SparkFlexControl {
    public CANSparkFlex motor;
    public RelativeEncoder motorEncoder;
    public SparkPIDController motorControl;

    /**
     * Create a Newton² controller for a NEO Vortex (with Spark Flex)
     *
     * @param motorCanID the motor's ID on the CAN bus
     * @param coastMode if true, the motor coasts when asked to apply 0 power.
     * Otherwise, it brakes.
     */
    public SparkFlexControl(int motorCanID, boolean coastMode){
        motor = new CANSparkFlex(motorCanID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();

        //Set update status to 20 Hz
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 50);

        motorControl = motor.getPIDController();
        motorEncoder = motor.getEncoder();
        if (coastMode){
            motor.setIdleMode(IdleMode.kCoast);
        }
        else{
            motor.setIdleMode(IdleMode.kBrake);
        }
        motor.set(0);
    }

    /**
     * Run the motor at a set velocity using the default PID slot
     *
     * @param RPM the velocity in RPM
     */
    public void setVelocity(double RPM){
        motorControl.setReference(RPM, com.revrobotics.CANSparkBase.ControlType.kVelocity);
    }

    /**
     * Run the motor at a set velocity using a specific PID slot
     *
     * @param RPM the velocity in RPM
     * @param slotID the PID slot to use
     */
    public void setVelocity(double RPM, int slotID){
        motorControl.setReference(RPM, com.revrobotics.CANSparkBase.ControlType.kVelocity, slotID);
    }

    /**
     * Drive the motor to a set position using the default PID slot
     *
     * @param rotations the target position in motor rotations
     */
    public void setPosition(double rotations){
        motorControl.setReference(rotations, com.revrobotics.CANSparkBase.ControlType.kPosition);
    }

    /**
     * Drive the motor with a set amount of power
     *
     * @param power the amount ofpower to apply, where -1 is full negative
     * power, 0 will apply the set neutral mode (brake or coast), and 1 applies
     * full forward power
     */
    public void setPercentOutput(double power){
        motor.set(power);
    }

    /**
     * Cut power to the motor hub and apply the set neutral mode (brake or coast)
     */
    public void stop(){
        motor.set(0);
    }

    public void setPIDF(double P, double I, double D, double FF, int slotID){
        motorControl =  motor.getPIDController();
        motorControl.setP(P,slotID);
        motorControl.setI(I, slotID);
        motorControl.setD(D, slotID);
        motorControl.setFF(FF, slotID);
        motorControl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal,slotID);
        motorControl.setSmartMotionAllowedClosedLoopError(0.02, slotID); // NOTE: THIS TOLERANCE IS IMPORTANT!!!!
    }

    /**
     * Returns the motor's velocity in RPM
     */
    public double getVelocity(){
        return motorEncoder.getVelocity();
    }

    /**
     * Returns the motor's position in rotations
     */
    public double getPosition(){
        return motorEncoder.getPosition();
    }

    /**
     * Invert the motor (positional setpoints, velocity setpoints,
     * the direction power mode drives it, etc)
     */
    public void setInverted(){
        motor.setInverted(true);
    }

    /**
     * Returns the motor's position in ticks
     */
    public double getTicks(){
        return motorEncoder.getPosition()*4096;
    }

    /**
     * Set a motor for this motor to mimmick the movements of.
     *
     * @param motorToFollow the motor for this one to follow
     */
    public void setFollower(SparkFlexControl motorToFollow){
        motor.follow(motorToFollow.motor);
    }

    /**
     * Set a motor for this motor to mimmick the movements of.
     *
     * @param sfc the motor for this one to follow
     * @param inverted whether to invert this motor's movements
     * relative to the motor it's following
     */
    public void follow(SparkFlexControl sfc, boolean inverted) {
        this.motor.follow(sfc.motor, inverted);
    }

    /**
     * Set a soft limit for the motor.
     *
     * @param direction the direction the motor must be traveling to
     * trigger the limit
     * @param rotations the number of rotations to limit the motor to
     */
    public void setSoftLimit(SoftLimitDirection direction, double rotations){
        motor.setSoftLimit(direction, (float)rotations);
    }

    /**
     * Set a target position for the motor to go to using a Smart
     * Motion profile.
     *
     * @param rotations the target position in rotations
     */
    public void setPositionSmartMotion(double rotations){
        motorControl.setReference(rotations, ControlType.kSmartMotion, 0);
    }

    /**
     * Set the maximum Smart Motion velocity.
     *
     * @param maxVelocity max velocity in RPM
     * @param slotID the Smart Motion PID slot
     */
    public void setMaxVelocity(double maxVelocity, int slotID){
        motorControl.setSmartMotionMaxVelocity(maxVelocity, slotID);
    }

    /**
     * Set the maximum Smart Motion acceleration.
     *
     * @param maxVelocity max velocity in RPM/s
     * @param slotID the Smart Motion PID slot
     */
    public void setMaxAcceleration(double maxAccel, int slotID){
        motorControl.setSmartMotionMaxAccel(maxAccel, slotID);
    }

    /**
     * Set the current limit of the motor.
     *
     * @param stallLimit the current limit when the motor is stalled
     * @param fullRPMLimit the current limit when at 5,700 RPM (the
     * motor's free speed)
     */
    public void setCurrentLimit(int stallLimit, int fullRPMLimit){
        this.motor.setSmartCurrentLimit(stallLimit, fullRPMLimit);
    }
}
