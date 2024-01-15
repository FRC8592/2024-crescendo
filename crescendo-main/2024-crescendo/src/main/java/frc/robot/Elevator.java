package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Elevator {
    private static final int ELEVATOR_MOTOR_CAN_ID = 0;
    private CANSparkFlex elevatorMotor;
    private SparkPIDController elevatorPID;
    private RelativeEncoder elevatorEncoder;

    public Elevator(){
        elevatorMotor = new CANSparkFlex(ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);
        elevatorPID = elevatorMotor.getPIDController();
        elevatorEncoder = elevatorMotor.getEncoder();
        
        elevatorPID.setP(0);
        elevatorPID.setI(0);
        elevatorPID.setD(0);
        elevatorPID.setFF(0);

        elevatorEncoder.setPosition(0);
    }

    public void percentOutputElevator(double speed){
        elevatorMotor.set(speed);
    }

    public void setPositionUp(double position){
        elevatorPID.setReference(position, ControlType.kPosition);
    }

    public void setPositionHome(double position){
        elevatorPID.setReference(position, ControlType.kPosition);
    }

    


}
