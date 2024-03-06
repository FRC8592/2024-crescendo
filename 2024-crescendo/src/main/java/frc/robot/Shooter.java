package frc.robot;
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

import com.revrobotics.SparkPIDController;

public class Shooter {
    
    // private final String shooterTableName = "shooter_table"; TODO: What are these three lines?
    // private final NetworkTable table;
    // private NetworkTableEntry shooterSpeedRPS;

    SparkFlexControl topShooterMotor;
    SparkFlexControl bottomShooterMotor;
    // SparkPIDController leftShooterControl;
    // SparkPIDController rightShooterControl;
    DigitalInput noteBeamBreak; // beam break sensor top/bottom mounted

    SparkFlexControl feederMotor;

    int toptargetSpeed = 0;
    int bottomTargetSpeed = 0;

    public boolean hasNote; 

    /**
     * Shooter object constructor
     */
    public Shooter() {
        topShooterMotor = new SparkFlexControl(SHOOTER.TOP_SHOOTER_MOTOR_CAN_ID, false);
        topShooterMotor.setInverted();
        bottomShooterMotor = new SparkFlexControl(SHOOTER.BOTTOM_SHOOTER_MOTOR_CAN_ID, false);
        feederMotor = new SparkFlexControl(SHOOTER.FEEDER_MOTOR_CAN_ID, false); 

        // table = NetworkTableInstance.getDefault().getTable(shooterTableName);
        topShooterMotor.setPIDF(SHOOTER.TOP_SHOOTER_MOTOR_kP, SHOOTER.TOP_SHOOTER_MOTOR_kI, SHOOTER.TOP_SHOOTER_MOTOR_kD, SHOOTER.TOP_SHOOTER_MOTOR_kF, 0);
        bottomShooterMotor.setPIDF(SHOOTER.BOTTOM_SHOOTER_MOTOR_kP, SHOOTER.BOTTOM_SHOOTER_MOTOR_kI, SHOOTER.BOTTOM_SHOOTER_MOTOR_kD, SHOOTER.BOTTOM_SHOOTER_MOTOR_kF, 0);

        feederMotor.setPIDF(SHOOTER.FEEDER_MOTOR_kP, SHOOTER.FEEDER_MOTOR_kI, SHOOTER.FEEDER_MOTOR_kD, SHOOTER.FEEDER_MOTOR_kF, 0);
        feederMotor.setInverted();

        bottomShooterMotor.setPercentOutput(0);
        topShooterMotor.setPercentOutput(0);
        feederMotor.setPercentOutput(0);

        noteBeamBreak = new DigitalInput(SHOOTER.NOTE_BEAM_BREAK_DIO_PORT);

        bottomShooterMotor.follow(topShooterMotor, true);

        topShooterMotor.motorControl.setIZone(SHOOTER.SHOOTER_MOTOR_IZONE);
        bottomShooterMotor.motorControl.setIZone(SHOOTER.SHOOTER_MOTOR_IZONE);
        // bottomShooterMotor.setInverted();
    }

    /**
     * set shooter motors speeds in terms of RPM
     * @param speedRPM
     */
    public void setShootVelocity(double speedRPM){
        topShooterMotor.setVelocity((int)speedRPM);
        bottomShooterMotor.setVelocity((int)speedRPM);
        toptargetSpeed = (int)speedRPM;
        bottomTargetSpeed = (int)speedRPM;
    }

    /**
     * set shooter motors speeds in terms of percent output
     * @param power
     */
    public void setShootPercentOutput(double power){
        topShooterMotor.setPercentOutput(power);
        bottomShooterMotor.setPercentOutput(power);
    }

    /**
     * Stops the flywheels
     */
    public void stop() {
        topShooterMotor.stop();
        bottomShooterMotor.stop();
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
     * @param feeder_velocity feeder velocity in RPM
    */
    public void setFeederVelocity(double feeder_velocity) {
        feederMotor.setVelocity(feeder_velocity);
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
    public boolean hasNote() {
        boolean currentlyHasNote = !noteBeamBreak.get(); // true if beam is BROKEN! thus note beam is not broken, no note
        if (currentlyHasNote) {
            hasNote = true;
        }
        return hasNote;
    }

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
    public boolean isReady() {
        SmartDashboard.putNumber("topShooterRPM", topShooterMotor.getVelocity());
        SmartDashboard.putNumber("bottomShooterRPM", bottomShooterMotor.getVelocity());
        if (Math.abs(topShooterMotor.getVelocity() - toptargetSpeed) < SHOOTER.FHYWHEEL_SPEED_ACCEPTABLE_RANGE&&
                Math.abs(bottomShooterMotor.getVelocity() - bottomTargetSpeed) < SHOOTER.FHYWHEEL_SPEED_ACCEPTABLE_RANGE) {
            hasNote = false;
            return true;
        }
        return false;
    }
}