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
    

    public void percentOutputElevator(double speed){
        elevatorMotor.set(speed);
    }

    public void setPosition(double position){
        elevatorPID.setReference(position, ControlType.kPosition);
    }

    public void setPositionAmp(){
        elevatorPID.setReference(POSITION_AMP ,ControlType.kPosition);
    }

    public void setPositionStart(double position){
        elevatorPID.setReference(POSITION_START, ControlType.kPosition);
    }
    
    /**
     * sets angle of elevator to base of robot
     * @param angle units: degrees
     */
    public void setAngle(double angle) {
        
    }

    /**
     * stows elevator and pivot 
     */
    public void stow() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stow'");
    }

    /**
     * gets position of  elevator
     * @return
     */
    public double getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

}
