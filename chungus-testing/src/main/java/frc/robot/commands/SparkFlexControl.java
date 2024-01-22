package frc.robot.commands;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

public class SparkFlexControl {
    CANSparkFlex motor;
    RelativeEncoder motorEncoder;
    SparkPIDController motorControl;


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

    public void setPercentOutput(double power){
        motor.set(power);
    }

    public void stop(){
        motor.set(0);
    }

    public void setPID(double P, double I, double D){
        motorControl =  motor.getPIDController();
        motorControl.setP(P);
        motorControl.setI(I);
        motorControl.setD(D);
        
    }

    public double getVelocity(){
        return motorEncoder.getVelocity();
    }
}
