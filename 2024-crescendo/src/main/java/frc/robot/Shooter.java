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

import com.revrobotics.SparkPIDController;

public class Shooter {
    
    // private final String shooterTableName = "shooter_table"; TODO: What are these three lines?
    // private final NetworkTable table;
    // private NetworkTableEntry shooterSpeedRPS;

    SparkFlexControl leftShooterMotor;
    SparkFlexControl rightShooterMotor;
    // SparkPIDController leftShooterControl;
    // SparkPIDController rightShooterControl;

    SparkFlexControl feederMotor;

    int targetSpeed = 0;

    public Shooter(){
        leftShooterMotor = new SparkFlexControl(SHOOTER.LEFT_SHOOTER_MOTOR_CAN_ID);
        leftShooterMotor.setInverted();
        rightShooterMotor = new SparkFlexControl(SHOOTER.RIGHT_SHOOTER_MOTOR_CAN_ID);

        // table = NetworkTableInstance.getDefault().getTable(shooterTableName);
        leftShooterMotor.setPIDF(SHOOTER.LEFT_SHOOTER_MOTOR_kP, SHOOTER.LEFT_SHOOTER_MOTOR_kI, SHOOTER.LEFT_SHOOTER_MOTOR_kD, 0);
        rightShooterMotor.setPIDF(SHOOTER.RIGHT_SHOOTER_MOTOR_kP, SHOOTER.RIGHT_SHOOTER_MOTOR_kI, SHOOTER.RIGHT_SHOOTER_MOTOR_kD, 0);

        rightShooterMotor.setPercentOutput(0);
        leftShooterMotor.setPercentOutput(0);
    }

    public void shootVelocity(double speedRPM){
        leftShooterMotor.setVelocity(speedRPM);
        rightShooterMotor.setVelocity(speedRPM);
    }

    public void shootPercentOutput(double power){
        leftShooterMotor.setPercentOutput(power);
        rightShooterMotor.setPercentOutput(power);
    }

    public void stop(){
        leftShooterMotor.stop();
        rightShooterMotor.stop();
    }

    /**
     * Sets speed of the feeder wheels
     * @param speed speed in rpm
     */
    public void setFeederSpeed(int speed) {
        leftShooterMotor.setPercentOutput(speed);
        rightShooterMotor.setPercentOutput(speed);
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
     * sets speed based on range table!!!
     * @param distanceToAprilTag
     */
    public void setSpeedRangeTable(double distanceToAprilTag) {

    }
    /**
     * checks if flywheels are at target speed to shoot
     */
    public boolean isReady() {
        if (leftShooterMotor.getVelocity() == targetSpeed && rightShooterMotor.getVelocity() == targetSpeed){
            return true;
        }
        return false;
    }
    /**
     * sets the alliance to blue or red!!
     * @param alliance
     */
    public void setAlliance(Alliance alliance) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAlliance'");
    }
}