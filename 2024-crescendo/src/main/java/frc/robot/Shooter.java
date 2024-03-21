package frc.robot;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.*;
//import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.SparkPIDController;

public class Shooter {
    
    // private final String shooterTableName = "shooter_table"; TODO: What are these three lines?
    // private final NetworkTable table;
    // private NetworkTableEntry shooterSpeedRPS;

    public SparkFlexControl leftShooterMotor;
    public SparkFlexControl rightShooterMotor;
    // SparkPIDController leftShooterControl;
    // SparkPIDController rightShooterControl;
    public DigitalInput bottomBeamBreak; // Detects the note when incoming
    public DigitalInput topBeamBreak; // Detects the note outgoing

    public SparkFlexControl feederMotor;

    int leftTargetSpeed = 0;
    int rightTargetSpeed = 0;

    public boolean hasNote = false;

    public enum States{
        INTAKING, //Unloaded (no note)
        RAM_TO_SHOOTERS, // Once we get a note, push the note into the shooter flywheels
        KEEP_RAMMING, // RAM_TO_SHOOTERS stops once the top beam break trips, so this runs for a moment to make sure the note is all the way up
        BACK_OFF_TO_SENSOR, // This state pulls the note down until the top sensor sees it
        BACK_OFF_FROM_SENSOR, // And this one continues to pull the note down until we don't see it anymore. The result is that
                              // the note ends up positioned just before the top beam-break.

        NOTHING,
        SHOOT,
        OUTAKE
    }

    private Timer shootTimer;
    private Timer keepRammingTimer;

    public States state;

    /**
     * Shooter object constructor
     */
    public Shooter() {
        leftShooterMotor = new SparkFlexControl(CAN.TOP_SHOOTER_MOTOR_CAN_ID, false);
        leftShooterMotor.setInverted();
        rightShooterMotor = new SparkFlexControl(CAN.BOTTOM_SHOOTER_MOTOR_CAN_ID, false);
        feederMotor = new SparkFlexControl(CAN.FEEDER_MOTOR_CAN_ID, false);

        // table = NetworkTableInstance.getDefault().getTable(shooterTableName);
        leftShooterMotor.setPIDF(SHOOTER.LEFT_SHOOTER_MOTOR_kP, SHOOTER.LEFT_SHOOTER_MOTOR_kI, SHOOTER.LEFT_SHOOTER_MOTOR_kD, SHOOTER.LEFT_SHOOTER_MOTOR_kF, 0);
        rightShooterMotor.setPIDF(SHOOTER.RIGHT_SHOOTER_MOTOR_kP, SHOOTER.RIGHT_SHOOTER_MOTOR_kI, SHOOTER.RIGHT_SHOOTER_MOTOR_kD, SHOOTER.RIGHT_SHOOTER_MOTOR_kF, 0);

        feederMotor.setPIDF(SHOOTER.FEEDER_MOTOR_kP, SHOOTER.FEEDER_MOTOR_kI, SHOOTER.FEEDER_MOTOR_kD, SHOOTER.FEEDER_MOTOR_kF, 0);
        feederMotor.setPIDF(SHOOTER.FEEDER_MOTOR_SHOOT_kP, SHOOTER.FEEDER_MOTOR_SHOOT_kI, SHOOTER.FEEDER_MOTOR_SHOOT_kD, SHOOTER.FEEDER_MOTOR_SHOOT_kF, 1);
        feederMotor.setInverted();

        rightShooterMotor.setCurrentLimit(POWER.RIGHT_SHOOTER_MOTOR_CURRENT_LIMIT, POWER.RIGHT_SHOOTER_MOTOR_CURRENT_LIMIT);
        leftShooterMotor.setCurrentLimit(POWER.LEFT_SHOOTER_MOTOR_CURRENT_LIMIT, POWER.LEFT_SHOOTER_MOTOR_CURRENT_LIMIT);
        feederMotor.setCurrentLimit(POWER.FEEDER_MOTOR_CURRENT_LIMIT, POWER.FEEDER_MOTOR_CURRENT_LIMIT);

        rightShooterMotor.setPercentOutput(0);
        leftShooterMotor.setPercentOutput(0);
        feederMotor.setPercentOutput(0);

        bottomBeamBreak = new DigitalInput(SHOOTER.BOTTOM_BEAM_BREAK_DIO_PORT);
        topBeamBreak = new DigitalInput(SHOOTER.TOP_BEAM_BREAK_DIO_PORT);

        leftShooterMotor.motorControl.setIZone(SHOOTER.SHOOTER_MOTOR_IZONE);
        rightShooterMotor.motorControl.setIZone(SHOOTER.SHOOTER_MOTOR_IZONE);

        shootTimer = new Timer();
        shootTimer.reset();
        shootTimer.stop();

        keepRammingTimer = new Timer();
        keepRammingTimer.reset();
        keepRammingTimer.stop();
        state = States.NOTHING;
    }

    public void update(){
        Logger.recordOutput(SHOOTER.LOG_PATH+"ShooterState", state.toString());
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/LeftTargetSpeed", leftTargetSpeed);
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/RightTargetSpeed", rightTargetSpeed);
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/RightShooterSpeed", rightShooterMotor.getVelocity());
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/LeftShooterSpeed", leftShooterMotor.getVelocity());
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/FeederSpeed", feederMotor.getVelocity());
        Logger.recordOutput(SHOOTER.LOG_PATH+"ReadyToShoot", readyToShoot());
        Logger.recordOutput(SHOOTER.LOG_PATH+"ShootTimer", shootTimer.get());
        Logger.recordOutput(SHOOTER.LOG_PATH+"KeepRammingTimer", keepRammingTimer.get());
        Logger.recordOutput(SHOOTER.LOG_PATH+"BottomBeamBreak", bottomBeamBreak.get());
        Logger.recordOutput(SHOOTER.LOG_PATH+"TopBeamBreak", topBeamBreak.get());
        switch (state) {
            default:
                break;
            case NOTHING:
                feederMotor.motorControl.setIAccum(0);
                break;
            case INTAKING:
                setShootVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED, SHOOTER.INTAKE_FLYWHEEL_SPEED);
                setFeederVelocity(SHOOTER.INTAKE_FEEDER_SPEED);
                if(!bottomBeamBreak.get()){ //Notice the exclamation point; the beam-break returns an inverted "is it tripped"
                state = States.RAM_TO_SHOOTERS;
            }
                break;
            case RAM_TO_SHOOTERS:
                setShootVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED, SHOOTER.INTAKE_FLYWHEEL_SPEED);
                feederMotor.setVelocity(SHOOTER.INTAKE_FEEDER_SPEED, 1);
                if(!topBeamBreak.get()){
                    keepRammingTimer.reset();
                    keepRammingTimer.start();
                    state = States.KEEP_RAMMING;
                }
                break;
            
            case KEEP_RAMMING:
                setShootVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED, SHOOTER.INTAKE_FLYWHEEL_SPEED);
                feederMotor.setVelocity(SHOOTER.INTAKE_FEEDER_SPEED, 1);
                if (keepRammingTimer.get() > SHOOTER.KEEP_RAMMING_TIME){
                    state = States.BACK_OFF_TO_SENSOR;
                }
                break;
                
            case BACK_OFF_TO_SENSOR:
                feederMotor.setVelocity(SHOOTER.ALIGN_SPEED, 1);
                if(!topBeamBreak.get()){
                    state = States.BACK_OFF_FROM_SENSOR;
                } 
                break;
                
            case BACK_OFF_FROM_SENSOR:
                feederMotor.setVelocity(SHOOTER.ALIGN_SPEED, 1);
                if(topBeamBreak.get()){
                    state = States.NOTHING;
                }

            case SHOOT:
                setShootVelocity(leftTargetSpeed, rightTargetSpeed);
                if(readyToShoot()){
                    feederMotor.setPercentOutput(SHOOTER.SHOOTING_FEEDER_POWER);
                    shootTimer.start();
                    if(shootTimer.hasElapsed(SHOOTER.SHOOT_SCORE_TIME)){
                        stop();
                        stopFeeders();
                        shootTimer.stop();
                        shootTimer.reset();
                        state = States.NOTHING;
                    }
                }
                break;
            case OUTAKE:
                setShootVelocity(SHOOTER.OUTAKE_FLYWHEEL_SPEED, SHOOTER.OUTAKE_FLYWHEEL_SPEED);
                setFeederVelocity(SHOOTER.OUTAKE_FEEDER_SPEED);
                if(bottomBeamBreak.get()){ // "If the top beam break doesn't see anything"
                    state = States.NOTHING;
                }
                break;
        }
    }

    /**
     * set shooter motors speeds in terms of RPM
     * @param speedRPM
     */
    public void setShootVelocity(int leftSpeedRPM, int rightSpeedRPM){
        leftTargetSpeed = leftSpeedRPM;
        rightTargetSpeed = rightSpeedRPM;
        leftShooterMotor.setVelocity(leftSpeedRPM);
        rightShooterMotor.setVelocity(rightSpeedRPM);
    }

    /**
     * set shooter motors speeds in terms of percent output
     * @param power
     */
    public void setShootPower(double power){
        leftShooterMotor.setPercentOutput(power);
        rightShooterMotor.setPercentOutput(power);
    }

    /**
     * Stops the flywheels
     */
    public void stop() {
        leftShooterMotor.stop();
        rightShooterMotor.stop();
    }

    /**
     * Sets speed of the feeder wheels in terms of percent output
     * @param power
     */
    public void setFeederPower(double power) {
        feederMotor.setPercentOutput(power);
    }

    /**
     * Sets velocity of the feeder wheels
     * @param feederVelocity feeder velocity in RPM
    */
    public void setFeederVelocity(double feederVelocity) {
        feederMotor.setVelocity(feederVelocity);
    }

    /**
     * stops feeder wheels from moving
     */
    public void stopFeeders() {
        feederMotor.stop();
    }

    /**
     * Checks whether note is in shooter
     * @return hasNote (boolean)
     */
    // public void hasNote() { //TODO: redo this function for the new shooter.
    //     boolean topBeamBreakReading = !topBeamBreak.get(); //beam break return the opposite of whether it sees something.
    //     boolean bottomBeamBreakReading = !bottomBeamBreak.get();
    //     if (state == States.INTAKING){
    //         hasNote = bottomBeamBreakReading;
    //     }
    //     else if (state == States.SHOOT){
    //         hasNote = topBeamBreakReading;
    //     }
    //     else if (state == States.OUTAKE){
    //         hasNote = bottomBeamBreakReading;
    //     }
    // }

    // /**
    //  * sets speed and angle based on range table
    //  * @param distanceToAprilTag
    //  */
    // public void setSpeedRangeTable(double distanceToAprilTag, Elevator elevator) {
    //     int index = (int)(distanceToAprilTag / CONVERSIONS.METERS_TO_FEET);
    //     double[] vals = SHOOTER.RANGE_TABLE[index];
    //     double angle = vals[0];
    //     int targetSpeed = (int)vals[1];
    //     // setShootVelocity(targetSpeed);/
    //     elevator.setPivotAngleCustom(angle);
    // }

    /**
     * checks if flywheels are at target speed to shoot
     * and resets hasNote cuz that means we're gonna shoot
     * 
     * @apiNote You must call this method before shooting
     */
    public boolean readyToShoot() {
        SmartDashboard.putNumber("topShooterRPM", leftShooterMotor.getVelocity());
        SmartDashboard.putNumber("bottomShooterRPM", rightShooterMotor.getVelocity());
        if (Math.abs(leftShooterMotor.getVelocity() - leftTargetSpeed) < SHOOTER.FHYWHEEL_SPEED_ACCEPTABLE_RANGE
                // && Math.abs(rightShooterMotor.getVelocity() - rightTargetSpeed) < SHOOTER.FHYWHEEL_SPEED_ACCEPTABLE_RANGE
                && leftTargetSpeed > 1000
                && rightTargetSpeed > 1000) {
            return true;
        }
        return false;
    }

    /**
     * Starts the intake state machine; run the feeders until we have a note, then back it off a little. All methods other than {@code intake()}, {@code hasNote()}, {@code isReady()}, and {@code intakeUpdate()} will cancel whatever the state machine is doing and prioritize that method's function.
     */
    public void intake(){
        this.state = States.INTAKING;
    }

    public void setTargetSpeed(int left, int right){
        leftTargetSpeed = left;
        rightTargetSpeed = right;
    }
    public void intakeStop(){
        state = States.NOTHING;
    }
    public void shoot(){
        feederMotor.motorControl.setIAccum(0);
        shootTimer.stop();
        shootTimer.reset();
        state = States.SHOOT;
    }
}