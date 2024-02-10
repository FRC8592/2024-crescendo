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

import com.revrobotics.SparkPIDController;

public class Shooter {
    
    // private final String shooterTableName = "shooter_table"; TODO: What are these three lines?
    // private final NetworkTable table;
    // private NetworkTableEntry shooterSpeedRPS;

    SparkFlexControl topShooterMotor;
    SparkFlexControl bottomShooterMotor;
    // SparkPIDController leftShooterControl;
    // SparkPIDController rightShooterControl;

    SparkFlexControl feederMotor;

    int toptargetSpeed = 0;
    int bottomTargetSpeed = 0;

    /**
     * Shooter object constructor
     */
    public Shooter() {
        topShooterMotor = new SparkFlexControl(SHOOTER.TOP_SHOOTER_MOTOR_CAN_ID);
        topShooterMotor.setInverted();
        bottomShooterMotor = new SparkFlexControl(SHOOTER.BOTTOM_SHOOTER_MOTOR_CAN_ID);
        feederMotor = new SparkFlexControl(SHOOTER.FEEDER_MOTOR_CAN_ID); 

        // table = NetworkTableInstance.getDefault().getTable(shooterTableName);
        topShooterMotor.setPIDF(SHOOTER.TOP_SHOOTER_MOTOR_kP, SHOOTER.TOP_SHOOTER_MOTOR_kI, SHOOTER.TOP_SHOOTER_MOTOR_kD, 0);
        bottomShooterMotor.setPIDF(SHOOTER.BOTTOM_SHOOTER_MOTOR_kP, SHOOTER.BOTTOM_SHOOTER_MOTOR_kI, SHOOTER.BOTTOM_SHOOTER_MOTOR_kD, 0);

        bottomShooterMotor.setPercentOutput(0);
        topShooterMotor.setPercentOutput(0);
        feederMotor.setPercentOutput(0);
    }

    /**
     * set shooter motors speeds in terms of RPM
     * @param speedRPM
     */
    public void setShootVelocity(int topspeedRPM, int bottomspeedRPM){
        topShooterMotor.setVelocity(topspeedRPM);
        bottomShooterMotor.setVelocity(bottomspeedRPM);
        toptargetSpeed = topspeedRPM;
        bottomTargetSpeed = bottomspeedRPM;
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
     * Sets speed of the feeder wheels
     * @param speed power
     */
    public void setFeederSpeed(double speed) {
        feederMotor.setPercentOutput(speed);
    }

    /**
     * stops feeder wheels from moving
     */
    public void stopFeeders() {
        feederMotor.stop();
    }

    /**
     * Checks whether note is in shooter
     * @return 
     */
    public boolean hasNote() {
        return true;
    }

    /**
     * sets speed and angle based on range table
     * @param distanceToAprilTag
     */
    public void setSpeedRangeTable(double distanceToAprilTag, Elevator elevator) {
        int index = (int)(distanceToAprilTag / CONVERSIONS.METERS_TO_FEET);
        double[] vals = SHOOTER.RANGE_TABLE[index];
        double angle = vals[0];
        int targetSpeed = (int)vals[1];
        // setShootVelocity(targetSpeed);/
        elevator.setAngle(angle);
    }

    /**
     * checks if flywheels are at target speed to shoot
     */
    public boolean isReady() {
        SmartDashboard.putNumber("leftShooterRPM", topShooterMotor.getVelocity());
        SmartDashboard.putNumber("rightShooterRPM", bottomShooterMotor.getVelocity());
        if (Math.abs(topShooterMotor.getVelocity() - toptargetSpeed) < SHOOTER.ACCEPTABLE_RANGE&&
                Math.abs(bottomShooterMotor.getVelocity() - bottomTargetSpeed) < SHOOTER.ACCEPTABLE_RANGE) {
            return true;
        }
        return false;
    }
}