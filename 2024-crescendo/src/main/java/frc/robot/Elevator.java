package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

public class Elevator {
    private SparkFlexControl extensionMotor;
    private SparkFlexControl pivotMotor;
    private SparkFlexControl pivotFollowMotor;
    private DigitalInput input;
    private DutyCycleEncoder absoluteEncoder;
    

    private double setAngle = ELEVATOR.PIVOT_ANGLE_STOWED;
    private double setLengthMeters = ELEVATOR.EXTENSION_METERS_STOWED;

    boolean pivotUp = false;
    boolean pivotDown = false;

    public Elevator(){
        extensionMotor = new SparkFlexControl(ELEVATOR.EXTENSION_MOTOR_CAN_ID, false);
        pivotMotor = new SparkFlexControl(ELEVATOR.PIVOT_MOTOR_CAN_ID, false);
        pivotFollowMotor = new SparkFlexControl(ELEVATOR.PIVOT_FOLLOW_MOTOR_CAN_ID, false);

        input = new DigitalInput(ELEVATOR.DIGITAL_INPUT_PORT_ABSOLUTE_ENCODER);
        absoluteEncoder = new DutyCycleEncoder(input);

        pivotMotor.setPIDF(ELEVATOR.PIVOT_kP, ELEVATOR.PIVOT_kI, ELEVATOR.PIVOT_kD, ELEVATOR.PIVOT_kFF, 0);
        pivotFollowMotor.setPIDF(ELEVATOR.PIVOT_kP, ELEVATOR.PIVOT_kI, ELEVATOR.PIVOT_kD, ELEVATOR.PIVOT_kFF, 0);

        extensionMotor.setPIDF(ELEVATOR.EXTENSION_kP, ELEVATOR.EXTENSION_kI, ELEVATOR.EXTENSION_kD, ELEVATOR.EXTENSION_kFF, 0);

        pivotMotor.setInverted();

        pivotMotor.setMaxVelocity(2000, 0);
        pivotFollowMotor.setMaxVelocity(2000, 0);

        extensionMotor.setMaxVelocity(2000, 0);

        pivotMotor.setMaxAcceleration(2000, 0);
        pivotFollowMotor.setMaxAcceleration(2000, 0);

        extensionMotor.setMaxAcceleration(2000, 0);
        
    }

    
    /**
     * Called in robot periodic - checks variables and updates motors
     */
    public void update(){
        double currentAngle = getPivotAngle(); // In degrees
        double currentLengthMeters = getExtensionLength(); // In ticks   
        
        boolean pivotIsRunning = false;
        boolean extensionIsRunning = false;

        SmartDashboard.putBoolean("Pivot down", pivotDown);
        SmartDashboard.putBoolean("Pivot up", pivotUp);
        
        boolean setPivot = false;
        boolean setExtend = false;

        if (pivotUp) {
            setPivot = currentAngle < ELEVATOR.PIVOT_ANGLE_MAX;
            setExtend = currentAngle > ELEVATOR.EXTENSION_ALLOWED_ANGLE && currentLengthMeters < ELEVATOR.EXTENSION_METERS_MAX;
        }
        else if (pivotDown){
            setExtend = currentLengthMeters > 0;
            setPivot = (currentAngle > ELEVATOR.EXTENSION_ALLOWED_ANGLE || currentLengthMeters < 0.01) && currentAngle>0;
        }
        else {
            setPivot = false;
            setExtend = false;
        }

        if (setPivot) {
            setPivotAngle(setAngle);
        }
        else {
            pivotMotor.stopSmartVelocity();
            pivotFollowMotor.stopSmartVelocity();
        }

        if (setExtend) {
            setExtensionLength(setLengthMeters);
        }
        else {
            extensionMotor.stopSmartVelocity();
        }

        // if(currentAngle<ELEVATOR.PIVOT_ANGLE_MAX && currentAngle>0){
        //     if(currentLengthMeters<ELEVATOR.EXTENSION_METERS_MAX && currentLengthMeters>0){
        //         pivotIsRunning = true;
        //         extensionIsRunning = true;
        //     } else{
        //         pivotIsRunning = true;
        //         extensionIsRunning = false;
        //     }
        // } else{
        //     if(currentLengthMeters<ELEVATOR.EXTENSION_METERS_MAX && currentLengthMeters>0){
        //         pivotIsRunning = false;
        //         extensionIsRunning = true;
        //     } else{
        //         pivotIsRunning = false;
        //         extensionIsRunning = false;
        //     }
        // }

        // if (pivotUp) {
        //     if (currentAngle > ELEVATOR.EXTENSION_ALLOWED_ANGLE /*&& currentLengthMeters<ELEVATOR.EXTENSION_METERS_MAX*/) { //30° is clear of all obstacles in the robot
        //         if(extensionIsRunning){
        //             setElevatorLength(setLengthMeters);
        //         } else{
        //             extensionMotor.stop();
        //         }
        //     }
        //     else {
        //         extensionMotor.stop();
        //     }

        //     if (pivotIsRunning){
        //         setPivotAngle(setAngle);
        //         } else{
        //             pivotMotor.stop();
        //             pivotFollowMotor.stop();
        //         }
            
        // }
        // else if (pivotDown) { //Moving down
        //     if (currentAngle > ELEVATOR.EXTENSION_ALLOWED_ANGLE /*&& currentAngle>0*/) { //Angle greater than 30° (no reference to elevator length)
        //         if (pivotIsRunning){
        //         setPivotAngle(setAngle);
        //         } else{
        //             pivotMotor.stop();
        //             pivotFollowMotor.stop();
        //         }
        //     }
        //     else if (currentLengthMeters > ELEVATOR.RETRACTED /*&& currentAngle>0*/) { // Elevator angle less than 30° and extended too far (more than 5 ticks)
        //         pivotMotor.stop();
        //         pivotFollowMotor.stop();
        //     }
        //     else { // elevator length <= 5 ticks AND angle less than 30°
        //         if (pivotIsRunning){
        //             setPivotAngle(setAngle);
        //             } else{
        //                 pivotMotor.stop();
        //                 pivotFollowMotor.stop();
        //             }
        //     }

        //     if(extensionIsRunning){
        //     setElevatorLength(setLengthMeters);
        //     } else{
        //         extensionMotor.stop();
        //     }
        // }
        // else { // Stopped
        //     extensionMotor.stop();
        //     pivotMotor.stop();
        //     pivotFollowMotor.stop();
        // }
    }

    //-------ELEVATOR CODE-------//

    /**
     * sets the speed of
     * @param speed
     */
    public void percentOutputExtension(double speed){
        extensionMotor.setPercentOutput(speed);
    }

    /** 
     * sets the position in rotations
     * @param position
    */
    private void setExtensionLength(double meters){ 
        double rotations = meters/ELEVATOR.ELEVATOR_GEAR_RATIO;
        extensionMotor.setPositionSmartMotion(rotations);
    }

    public void setExtensionLengthCustom(double position){
        setLengthMeters = position;
    }

    /**
     * gets length of elevator in meters
     * @return
     */
    public double getExtensionLength() {
        return (extensionMotor.getPosition()*ELEVATOR.ELEVATOR_GEAR_RATIO);
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
    private void setPivotAngle(double angle) {
        double rotations = (ELEVATOR.PIVOT_GEAR_RATIO * angle)/360;
        pivotMotor.setPositionSmartMotion(rotations);
        pivotFollowMotor.setPositionSmartMotion(rotations);
    }

    public void setPivotAngleCustom(double angle) {
        setAngle = angle;
        
        pivotUp = getPivotAngle() < setAngle;
        pivotDown = getPivotAngle() > setAngle;
    }

    /**
     * gets position of pivot
     * @return
     */
    public double getPivotAngle() {
        double ticksConverted = (pivotMotor.getTicks()*CONVERSIONS.TICKS_TO_ANGLE_DEGREES)/ELEVATOR.PIVOT_GEAR_RATIO;
        return ticksConverted;
    }

    //-------COMBINED CODE-------//

    /**
     * stows elevator and pivot 
     */
    public void stow() {
        setLengthMeters = ELEVATOR.EXTENSION_METERS_STOWED;
        setAngle = ELEVATOR.PIVOT_ANGLE_STOWED;

        pivotUp = getPivotAngle() < setAngle;
        pivotDown = getPivotAngle() > setAngle;
    }

    public void ampPosition() {
        setLengthMeters = ELEVATOR.EXTENSION_METERS_AMP;
        setAngle = ELEVATOR.PIVOT_ANGLE_AMP;

        pivotUp = getPivotAngle() < setAngle;
        pivotDown = getPivotAngle() > setAngle;
    }

    public void climbPosition(){
        setLengthMeters = ELEVATOR.EXTENSION_METERS_CLIMB;
        setAngle = ELEVATOR.PIVOT_ANGLE_CLIMB;


        pivotUp = getPivotAngle() < setAngle;
        pivotDown = getPivotAngle() > setAngle;
    }

    /**
     * gets the value of the absolute encoder
     * @return position
     */
    public double getAbsoluteValueEncoderPosition(){
        return absoluteEncoder.getAbsolutePosition();
    }

    /**
     * gets the value of the absolute encoder
     */
    public void zeroAbsoluteValueEncoder(){
        absoluteEncoder.reset();
    }
    
    /**
     * resets the encoders for pivot motor, pivot follow motor, extension motor
    */
    public void resetEncoders() {
        pivotMotor.motorEncoder.setPosition(0);
        pivotFollowMotor.motorEncoder.setPosition(0);
        extensionMotor.motorEncoder.setPosition(0);
    }
    
    public void testLimits(){
        setLengthMeters = 2;
        setAngle = 100;

        pivotUp = getPivotAngle() < setAngle;
        pivotDown = getPivotAngle() > setAngle;
    }

}
