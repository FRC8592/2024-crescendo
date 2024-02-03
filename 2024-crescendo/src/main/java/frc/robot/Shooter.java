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
    RelativeEncoder leftShooterEncoder;
    RelativeEncoder rightShooterEncoder;
    SparkPIDController leftShooterControl;
    SparkPIDController rightShooterControl;

    CANSparkFlex feederMotor;
    RelativeEncoder feederEncoder;
    SparkPIDController feederControl;

    public Shooter(){
        leftShooterMotor = new SparkFlexControl(Constants.LEFT_SHOOTER_MOTOR_CAN_ID);
        leftShooterMotor.setInverted(true);
        rightShooterMotor = new SparkFlexControl(Constants.RIGHT_SHOOTER_MOTOR_CAN_ID);

        // table = NetworkTableInstance.getDefault().getTable(shooterTableName);
        leftShooterControl =  leftShooterMotor.getPIDController();
        rightShooterControl = rightShooterMotor.getPIDController();
        leftShooterControl.setP(SHOOTER.LEFT_SHOOTER_MOTOR_kP); //PID
        leftShooterControl.setI(SHOOTER.LEFT_SHOOTER_MOTOR_kI); //PID
        leftShooterControl.setD(SHOOTER.LEFT_SHOOTER_MOTOR_kD); //PID
        rightShooterControl.setP(SHOOTER.RIGHT_SHOOTER_MOTOR_kP); //PID
        rightShooterControl.setI(SHOOTER.RIGHT_SHOOTER_MOTOR_kI); //PID
        rightShooterControl.setD(SHOOTER.RIGHT_SHOOTER_MOTOR_kD); //PID

        rightShooterMotor.set(0);
        leftShooterMotor.set(0);
    }

    public void shootVelocityMode(double speedRPM){
        leftShooterControl.setReference(speedRPM, com.revrobotics.CANSparkBase.ControlType.kVelocity); 
        rightShooterControl.setReference(speedRPM, com.revrobotics.CANSparkBase.ControlType.kVelocity);
    }

    public void shootPercentOutput(double power){
        leftShooterMotor.set(Math.min(Math.max(power,-0.5),0.5)); 
        rightShooterMotor.set(Math.min(Math.max(power,-0.5),0.5));
    }

    public void stop(){
        leftShooterMotor.setVelocity(0);
        rightShooterMotor.set(0);
    }

    /**
     * Sets speed of the feeder wheels
     * @param speed speed in rpm
     */
    public void setFeederSpeed(int speed) {
        leftShooterMotor.set(speed);
        rightShooterMotor.set(speed);
    }
    
    /**
     * Spins feeder motors to shoot note
     */
    public void shoot() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopFeeders'");
    }

    /**
     * Checks whether note is in shooter
     * @return 
     */
    public boolean hasNote() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopFeeders'");
    }
    
    /**
     * stops flywheels from moving!
     */
    public void stopFlywheels() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopFeeders'");

    }
    
    /**
     * stops feeder wheels from moving uwu
     */
    public void stopFeeders() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopFeeders'");
    }
    /**
     * sets speed based on range table!!!
     * @param distanceToAprilTag
     */
    public void setSpeedRangeTable(double distanceToAprilTag) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSpeedRangeTable'");
        double targetSpeed = 
    }
    /**
     * checks if flywheels are at target speed to shoot
     * @return
     */
    public boolean isReady() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isReady'");
        if (leftShooterMotor.getVelocity() == targetSpeed && rightShooterMotor.getVelocity() == targetSpeed){
            return true;
        }
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