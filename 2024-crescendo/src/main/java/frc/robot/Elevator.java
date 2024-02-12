package frc.robot;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

public class Elevator {
    private SparkFlexControl extensionMotor;
    private SparkFlexControl pivotMotor;
    private SparkFlexControl pivotFollowMotor;
    private AbsoluteEncoder pivotEncoder;

    private double setAngle = ELEVATOR.PIVOT_ANGLE_STOWED;
    private double setLengthTicks = ELEVATOR.POSITION_STOWED;


    public Elevator(){
        extensionMotor = new SparkFlexControl(ELEVATOR.EXTENSION_MOTOR_CAN_ID);
        pivotMotor = new SparkFlexControl(ELEVATOR.PIVOT_MOTOR_CAN_ID);
        pivotFollowMotor = new SparkFlexControl(ELEVATOR.PIVOT_FOLLOW_MOTOR_CAN_ID);

        pivotMotor.setPIDF(ELEVATOR.PIVOT_kP, ELEVATOR.PIVOT_kI, ELEVATOR.PIVOT_kD, ELEVATOR.PIVOT_kFF);
        pivotFollowMotor.setPIDF(ELEVATOR.PIVOT_kP, ELEVATOR.PIVOT_kI, ELEVATOR.PIVOT_kD, ELEVATOR.PIVOT_kFF);

        pivotMotor.setInverted();
        
        //pivotFollowMotor.setInverted();
        
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
     * sets the position in rotations
     * @param position
    */
    private void setElevatorLength(double position){ 
        extensionMotor.setPosition(position);
    }

    public void setElevatorLengthCustom(double position){
        setLengthTicks = position;
    }

    /**
     * gets length of elevator in meters
     * @return
     */
    public double getElevatorLength() {
        return extensionMotor.getPosition();
    }

    //-------PIVOT CODE-------//

    /**
     * sets the speed of
     * @param speed
     */
    public void percentOutputPivot(double speed){
        pivotMotor.setPercentOutput(speed);
        pivotFollowMotor.setPercentOutput(speed);
    }
    
    /**
     * sets angle of pivot to base of robot
     * @param angle units: degrees
     */
    public void setPivotAngle(double angle) {
        double rotations = (CONVERSIONS.PIVOT_GEAR_RATIO * angle)/360;
        pivotMotor.setPosition(rotations);
        pivotFollowMotor.setPosition(rotations);
        SmartDashboard.putNumber("rotations in setPivotAngle", rotations);
    }

    public void setPivotAngleCustom(double angle) {
        setAngle = angle;
    }

    /**
     * gets position of pivot
     * @return
     */
    public double getPivotAngle() {
        double ticksConverted = (pivotMotor.getTicks()*CONVERSIONS.TICKS_TO_ANGLE_DEGREES)/CONVERSIONS.PIVOT_GEAR_RATIO;
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

    public void zero(){
        pivotEncoder.setZeroOffset(pivotEncoder.getPosition());
    }

    public void resetEncoders(){
        pivotMotor.motorEncoder.setPosition(0);
        pivotFollowMotor.motorEncoder.setPosition(0);
        extensionMotor.motorEncoder.setPosition(0);
    }

}
