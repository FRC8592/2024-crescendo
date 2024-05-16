package frc.robot;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
     * Create a Newton2 controller for a NEO Vortex (with Spark Flex)
     * @param motorCanID
     * @param coastMode If true, the motor coasts when asked to apply 0 power. Otherwise, it brakes.
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
        } else{
            motor.setIdleMode(IdleMode.kBrake);
        }
        motor.set(0);
    }

    public void setVelocity(double RPM){
        motorControl.setReference(RPM, com.revrobotics.CANSparkBase.ControlType.kVelocity); 
    }

    public void setVelocity(double RPM, int slotID){
        motorControl.setReference(RPM, com.revrobotics.CANSparkBase.ControlType.kVelocity, slotID); 
    }

    public void setPosition(double rotations){
        motorControl.setReference(rotations, com.revrobotics.CANSparkBase.ControlType.kPosition); 
    }

    public void setPercentOutput(double power){
        motor.set(power);
    }

    public void stop(){
        motor.set(0);
    }

    public void stopSmartVelocity(){
        motorControl.setReference(0, ControlType.kSmartVelocity, 0);
    }

    public void setPIDF(double P, double I, double D, double FF, int slotID){
        motorControl =  motor.getPIDController();
        motorControl.setP(P,slotID);
        motorControl.setI(I, slotID);
        motorControl.setD(D, slotID);
        motorControl.setFF(FF, slotID);
        motorControl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal,slotID);
        motorControl.setSmartMotionAllowedClosedLoopError(0.02, slotID); // NOTE: THIS TOLERNACE IS IMPORTANT!!!!
    }

    public double getVelocity(){
        return motorEncoder.getVelocity();
    }
    
    public double getPosition(){
        return motorEncoder.getPosition();
    }

    public void setInverted(){
        motor.setInverted(true);
    }

    public double getTicks(){
        return motorEncoder.getPosition()*4096;
    }


    public void setFollower(SparkFlexControl motorToFollow){
        motor.follow(motorToFollow.motor);
    }

    public void setSoftLimit(SoftLimitDirection direction, double rotations){
        motor.setSoftLimit(direction, (float)rotations);
    }

    public void setPositionSmartMotion(double rotations){
        motorControl.setReference(rotations, ControlType.kSmartMotion, 0);
        // SmartDashboard.putNumber("smart motion", rotations);
    }

    public void setMaxVelocity(double maxVelocity, int slotID){
        motorControl.setSmartMotionMaxVelocity(maxVelocity, slotID);
    }
    public void setMaxAcceleration(double maxAccel, int slotID){
        motorControl.setSmartMotionMaxAccel(maxAccel, slotID);
    }
    public void follow(SparkFlexControl sfc, boolean inverted) {
        this.motor.follow(sfc.motor, inverted);
    }
    public void setCurrentLimit(int stallLimit, int fullRPMLimit){
        this.motor.setSmartCurrentLimit(stallLimit, fullRPMLimit);
    }
}
