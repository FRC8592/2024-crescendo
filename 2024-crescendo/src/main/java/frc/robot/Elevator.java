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
    private final double POSITION_START = 0;
    private final double POSITION_AMP= 0;

    public Elevator(){
        elevatorMotor = new CANSparkFlex(ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);
        elevatorPID = elevatorMotor.getPIDController();
        elevatorEncoder = elevatorMotor.getEncoder();
        
        elevatorPID.setP(0, 0);
        elevatorPID.setI(0, 0);
        elevatorPID.setD(0, 0);
        elevatorPID.setFF(0, 0);

        elevatorEncoder.setPosition(0);
    }
    
    /**
     * Sets the speed of the elevator to the speed given
     * @param speed
     */
    public void percentOutputElevator(double speed){
        elevatorMotor.set(speed);
    }
    /**
     * Sets the position of the elevator based on the ticks it is given
     * @param position
     */
    public void setPosition(double position){
        elevatorPID.setReference(position, ControlType.kPosition);
    }
    /**
     * sets the position of the elevator and pivot to be able to shoot from the amp
     */
    public void setPositionAmp(){
        elevatorPID.setReference(POSITION_AMP ,ControlType.kPosition);
    }
    /**
     * Sets the position of the elevator and pivot to be able to shoot from the lowest position
     * @param
     */
    public void setPositionStart(){
        elevatorPID.setReference(POSITION_START, ControlType.kPosition);
    }
    


}
