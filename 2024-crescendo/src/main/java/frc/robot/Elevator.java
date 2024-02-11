package frc.robot;
import frc.robot.Constants.*;

public class Elevator {
    private SparkFlexControl extensionMotor;
    private SparkFlexControl pivotMotor;
    private SparkFlexControl pivotFollowMotor;

    private double setAngle = ELEVATOR.PIVOT_ANGLE_STOWED;
    private double setLengthTicks = ELEVATOR.POSITION_STOWED;


    public Elevator(){
        extensionMotor = new SparkFlexControl(ELEVATOR.EXTENSION_MOTOR_CAN_ID);
        pivotMotor = new SparkFlexControl(ELEVATOR.PIVOT_MOTOR_CAN_ID);
        pivotFollowMotor = new SparkFlexControl(ELEVATOR.PIVOT_FOLLOW_MOTOR_CAN_ID);

        pivotFollowMotor.setFollower(pivotMotor);
    }


    public void update(){
        double currentAngle = getPivotAngle(); // In degrees
        double currentLengthTicks = getElevatorLength(); // In ticks
        boolean pivotUp = false;
        boolean pivotDown = false;

        pivotUp = currentAngle < setAngle;
        pivotDown = currentAngle > setAngle;
        if (pivotUp) {
            if (currentAngle > ELEVATOR.LIFTED) { //30° is clear of all obstacles in the robot
                setElevatorLength(setLengthTicks); //Position == length
            }
            else {
                extensionMotor.stop();
            }
            setPivotAngle(setAngle);
        }
        else if (pivotDown) { //Moving down
            if (currentAngle > ELEVATOR.LIFTED) { //Angle greater than 30° (no reference to elevator length)
                setPivotAngle(setAngle);
            }
            else if (currentLengthTicks > ELEVATOR.RETRACTED) { // Elevator angle less than 30° and extended too far (more than 5 ticks)
                pivotMotor.stop();
            }
            else { // elevator length <= 5 ticks AND angle less than 30°
                setPivotAngle(setAngle);
            }
            setElevatorLength(setLengthTicks);
        }
        else { // Stopped
            extensionMotor.stop();
            pivotMotor.stop();
        }
    }

    //-------ELEVATOR CODE-------//

    /**
     * sets the speed of
     * @param speed
     */
    public void percentOutputElevator(double speed){
        extensionMotor.setPercentOutput(speed);
    }

    /** 
     * sets the position
    */
    private void setElevatorLength(double position){
        extensionMotor.setPosition(position);
    }

    public void setElevatorLengthCustom(double position){
        setLengthTicks = position;
    }

    /**
     * gets position of elevator
     * @return
     */
    public double getElevatorLength() {
        return extensionMotor.getTicks();
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
    private void setPivotAngle(double angle) {
        double angleConverted = CONVERSIONS.ANGLE_DEGREES_TO_TICKS * angle;
        pivotMotor.setPosition(angleConverted);
    }

    public void setPivotAngleCustom(double angle) {
        setAngle = angle;
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
        setLengthTicks = ELEVATOR.POSITION_STOWED;
        setAngle = ELEVATOR.PIVOT_ANGLE_STOWED;
    }

    public void ampPosition() {
        setLengthTicks = ELEVATOR.POSITION_AMP;
        setAngle = ELEVATOR.PIVOT_ANGLE_AMP;
    }

    public void climbPosition(){
        setLengthTicks = ELEVATOR.POSITION_CLIMB;
        setAngle = ELEVATOR.PIVOT_ANGLE_CLIMB;
    }

}
