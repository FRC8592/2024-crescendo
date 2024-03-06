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
    SparkFlexControl topShooterMotor;
    SparkFlexControl bottomShooterMotor;
    DigitalInput noteBeamBreak; // beam break sensor

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
    }

    /**
     * set shooter motors speeds in terms of RPM
     * @param speedRPM
     */
    public void setFlywheelVelocity(double speedRPM){
        topShooterMotor.setVelocity((int)speedRPM);
        bottomShooterMotor.setVelocity((int)speedRPM);
        toptargetSpeed = (int)speedRPM;
        bottomTargetSpeed = (int)speedRPM;
    }

    /**
     * set shooter motors speeds in terms of percent output
     * @param power
     */
    public void setFlywheelPower(double power){
        topShooterMotor.setPercentOutput(power);
        bottomShooterMotor.setPercentOutput(power);
    }

    /**
     * Stops the flywheels
     */
    public void stopFlywheels() {
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
    public boolean hasNote() {
        boolean currentlyHasNote = !noteBeamBreak.get(); // true if beam is BROKEN! thus note beam is not broken, no note
        if (currentlyHasNote) {
            hasNote = true;
        }
        return hasNote;
    }

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