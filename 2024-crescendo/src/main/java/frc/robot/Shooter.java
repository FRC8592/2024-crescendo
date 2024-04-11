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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

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
    public DigitalInput middleBeamBreak;

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
        SHOOT_PREP,
        NOTHING,
        SHOOT,
        OUTAKE
    }

    private Timer shootTimer;
    private Timer keepRammingTimer;
    private Timer rumbleTimer;

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
        feederMotor.setPIDF(SHOOTER.FEEDER_MOTOR_LOADED_kP, SHOOTER.FEEDER_MOTOR_LOADED_kI, SHOOTER.FEEDER_MOTOR_LOADED_kD, SHOOTER.FEEDER_MOTOR_LOADED_kF, 1);
        feederMotor.motorControl.setIZone(SHOOTER.FEEDER_MOTOR_LOADED_IZONE, 1);
        feederMotor.setPIDF(SHOOTER.FEEDER_MOTOR_OUTAKE_kP, SHOOTER.FEEDER_MOTOR_OUTAKE_kI, SHOOTER.FEEDER_MOTOR_OUTAKE_kD, SHOOTER.FEEDER_MOTOR_OUTAKE_kF, 2);
        feederMotor.motorControl.setIZone(SHOOTER.FEEDER_MOTOR_OUTAKE_IZONE, 2);
        feederMotor.setInverted();

        rightShooterMotor.setCurrentLimit(POWER.RIGHT_SHOOTER_MOTOR_CURRENT_LIMIT, POWER.RIGHT_SHOOTER_MOTOR_CURRENT_LIMIT);
        leftShooterMotor.setCurrentLimit(POWER.LEFT_SHOOTER_MOTOR_CURRENT_LIMIT, POWER.LEFT_SHOOTER_MOTOR_CURRENT_LIMIT);
        feederMotor.setCurrentLimit(POWER.FEEDER_MOTOR_CURRENT_LIMIT, POWER.FEEDER_MOTOR_CURRENT_LIMIT);

        rightShooterMotor.setPercentOutput(0);
        leftShooterMotor.setPercentOutput(0);
        feederMotor.setPercentOutput(0);

        bottomBeamBreak = new DigitalInput(SHOOTER.BOTTOM_BEAM_BREAK_DIO_PORT);
        topBeamBreak = new DigitalInput(SHOOTER.TOP_BEAM_BREAK_DIO_PORT);
        middleBeamBreak = new DigitalInput(SHOOTER.MIDDLE_BEAM_BREAK_DIO_PORT);

        leftShooterMotor.motorControl.setIZone(SHOOTER.SHOOTER_MOTOR_IZONE);
        rightShooterMotor.motorControl.setIZone(SHOOTER.SHOOTER_MOTOR_IZONE);

        shootTimer = new Timer();
        shootTimer.reset();
        shootTimer.stop();

        rumbleTimer = new Timer();
        rumbleTimer.reset();
        rumbleTimer.stop();

        keepRammingTimer = new Timer();
        keepRammingTimer.reset();
        keepRammingTimer.stop();
        state = States.NOTHING;
    }

    public void log() {
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/LeftTargetSpeed", leftTargetSpeed);
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/RightTargetSpeed", rightTargetSpeed);
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/RightShooterSpeed", rightShooterMotor.getVelocity());
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/LeftShooterSpeed", leftShooterMotor.getVelocity());
        Logger.recordOutput(SHOOTER.LOG_PATH+"MotorRPMs/FeederSpeed", feederMotor.getVelocity());
        Logger.recordOutput(SHOOTER.LOG_PATH+"ReadyToShoot", readyToShoot());
        Logger.recordOutput(SHOOTER.LOG_PATH+"BottomBeamBreak", bottomBeamBreak.get());
        Logger.recordOutput(SHOOTER.LOG_PATH+"TopBeamBreak", topBeamBreak.get());
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
    public void stopFlywheels() {
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
     * Sets velocity of the feeder wheels
     * @param feederVelocity feeder velocity in RPM
     * @param slotID PID controller slot to pass to the motor
    */
    public void setFeederVelocity(double feederVelocity, int slotID) {
        feederMotor.setVelocity(feederVelocity, slotID);
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

    public boolean isBottomBeamBreakTripped(){
        return !bottomBeamBreak.get(); //The beam break pulls low when triggered (notice exclamation point)
    }

    public boolean isTopBeamBreakTripped(){
        return !topBeamBreak.get(); //The beam break pulls low when triggered (notice exclamation point)
    }

    public boolean isMiddleBeamBreakTripped(){
        return !middleBeamBreak.get(); //The beam break pulls low when triggered (notice exclamation point)
    }
}