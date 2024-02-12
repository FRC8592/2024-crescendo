package frc.robot;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

public class SparkFlexControl {
    public CANSparkFlex motor;
    public RelativeEncoder motorEncoder;
    public SparkPIDController motorControl;


    public SparkFlexControl(int MOTOR_CAN_ID){
        motor = new CANSparkFlex(MOTOR_CAN_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motorControl = motor.getPIDController();
        motorEncoder = motor.getEncoder();
        motor.set(0);
    }

    public void setVelocity(double RPM){
        motorControl.setReference(RPM, com.revrobotics.CANSparkBase.ControlType.kVelocity); 
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

    public void setPIDF(double P, double I, double D, double FF){
        motorControl =  motor.getPIDController();
        motorControl.setP(P);
        motorControl.setI(I);
        motorControl.setD(D);
        motorControl.setFF(FF);
    }

    public double getVelocity(){
        return motorEncoder.getVelocity();
    }

    public double getPosition() {
        return motorEncoder.getPosition();
    }

    public double getTicks(){
        return motorEncoder.getPosition()*4096;
    }


    public void setFollower(SparkFlexControl motorToFollow){
        motor.follow(motorToFollow.motor);
    }

    public void setInverted(){
        motor.setInverted(true);
    }

    public void setSoftLimit(SoftLimitDirection direction, double rotations){
        motor.setSoftLimit(direction, (float)rotations);
    }
}
