package frc.robot;
import frc.robot.Constants.*;

public class Elevator {
    private SparkFlexControl elevatorMotor;
    private SparkFlexControl pivotMotor;
    private SparkFlexControl pivotFollowMotor;


    public Elevator(){
        elevatorMotor = new SparkFlexControl(ELEVATOR.ELEVATOR_MOTOR_CAN_ID);
        pivotMotor = new SparkFlexControl(ELEVATOR.PIVOT_MOTOR_CAN_ID);
        pivotFollowMotor = new SparkFlexControl(ELEVATOR.PIVOT_FOLLOW_MOTOR_CAN_ID);

        pivotFollowMotor.setFollower(pivotMotor);
    }

    private double setAngle = ELEVATOR.PIVOT_ANGLE_STOWED;
    private double setTicks = ELEVATOR.POSITION_STOWED;

    public void elevatorPeriodic(){
        double currAngle = getPivotAngle();
        double currTicks = getElevatorPosition();
        boolean up = true;


        if(currAngle > setAngle){
            up = false;
        } else{
            up = true;
        }

        if (currAngle > 30 || currTicks < 5){ //30 = the angle we need to stop and retract at, 5 = if the elvator is almost fully retracted or not
            setPivotAngle(setAngle);
            setElevatorPosition(setTicks);
        } else if(currAngle<30 && up == true){
            setPivotAngle(setAngle);
            setElevatorPosition(setTicks);
        } else{
            elevatorMotor.stop();
            pivotMotor.stop();
        }
        
    }

    //-------ELEVATOR CODE-------//

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
    public void setElevatorPosition(double position){
        elevatorMotor.setPosition(position);
    }

    public void setElevatorPositionCustom(double position){
        setTicks = position;
    }

    /**
     * sets the position of the elevator and pivot to shoot in amp
     */
    public void setElevatorPositionAmp(){
        setElevatorPosition(ELEVATOR.POSITION_AMP);
    }

    /**
     * sets elevator and pivot position to shooting from stowed position
     * @param position 
     * */
    
     public void setElevatorPositionStowed(){
        setElevatorPosition(ELEVATOR.POSITION_STOWED);
    }

    /**
     * gets position of elevator
     * @return
     */
    public double getElevatorPosition() {
        return elevatorMotor.getTicks();
    }

    //-------PIVOT CODE-------//

    /**
     * sets the speed of
     * @param speed
     */
    public void percentOutputPivot(double speed){
        pivotMotor.setPercentOutput(speed);
    }
    
    /**
     * sets angle of pivot to base of robot
     * @param angle units: degrees
     */
    public void setPivotAngle(double angle) {
        double angleConverted = CONVERSIONS.ANGLE_DEGREES_TO_TICKS * angle;
        pivotMotor.setPosition(angleConverted);
    }

    public void setPivotAngleCustom(double angle) {
        setAngle = angle;
    }

    /**
     * Sets the position of the pivot to the correct angle to stow it
     */

     public void setPivotAngleStowed() {
        setPivotAngle(ELEVATOR.PIVOT_ANGLE_STOWED);
    }

    /**
     * Sets the position of the pivot to the correct angle to score in the amp
     */

    public void setPivotAngleAmp() {
        setPivotAngle(ELEVATOR.PIVOT_ANGLE_AMP);
    }

    /**
     * gets position of pivot
     * @return
     */
    public double getPivotAngle() {
        double ticksConverted = CONVERSIONS.TICKS_TO_ANGLE_DEGREES*pivotMotor.getTicks();
        return ticksConverted;
    }

    //-------COMBINED CODE-------//

    /**
     * stows elevator and pivot 
     */
    public void stow() {
        setElevatorPositionStowed();;
        if (Math.abs(elevatorMotor.getPosition()) <= 100){
            setPivotAngleStowed();
        }
    }

}
