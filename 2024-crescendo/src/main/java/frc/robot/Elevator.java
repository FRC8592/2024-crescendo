package frc.robot;
import frc.robot.Constants.*;

public class Elevator {
    private SparkFlexControl elevatorMotor;
    private SparkFlexControl pivotMotor;


    public Elevator(){
        elevatorMotor = new SparkFlexControl(ELEVATOR.ELEVATOR_MOTOR_CAN_ID);
        pivotMotor = new SparkFlexControl(ELEVATOR.PIVOT_MOTOR_CAN_ID);
    }
    
    /**
     * sets the speed of
     * @param speed
     */
    public void percentOutputElevator(double speed){
        elevatorMotor.setPercentOutput(speed);
    }

    /** 
     * sets the position
    */
    public void setPosition(double position){
        elevatorMotor.setVelocity(position);
    }

    /**
     * sets the position of the elevator and pivot to shoot in amp
     */
    public void setPositionAmp(){
        elevatorMotor.setVelocity(ELEVATOR.POSITION_AMP);
    }

    /**
     * sets elevator and pivot position to shooting from stowed position
     * @param position 
     * */
    
     public void setPositionStart(double position){
        elevatorMotor.setVelocity(ELEVATOR.POSITION_START);
    }
    
    /**
     * sets angle of pivot to base of robot
     * @param angle units: degrees
     */
    public void setPivotAngle(double angle) {
        double angleConverted = CONVERSIONS.ANGLE_DEGREES_TO_TICKS * angle;
        pivotMotor.setPosition(angleConverted);
    }

    /**
     * stows elevator and pivot 
     */
    public void stow() {
        elevatorMotor.setPosition(0);
        if (Math.abs(elevatorMotor.getPosition()) <= 100){
            pivotMotor.setPosition(0);
        }
    }

    /**
     * gets position of elevator
     * @return
     */
    public double getElevatorPosition() {
        return elevatorMotor.getPosition();
    }

    /**
     * gets position of pivot
     * @return
     */
    public double getPivotPosition() {
        return pivotMotor.getPosition();
    }

    public void setPivotAmp() {
        setPivotAngle(ELEVATOR.PIVOT_ANGLE_AMP);
    }

}
