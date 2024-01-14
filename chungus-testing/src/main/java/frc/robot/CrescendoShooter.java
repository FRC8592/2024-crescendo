package frc.robot;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.SparkPIDController;

public class CrescendoShooter {
    // Constants
    public static final int LEFT_MOTOR_CAN_ID = 31;
    public static final int RIGHT_MOTOR_CAN_ID = 29;

    private final String shooterTableName = "shooter_table";
    private final NetworkTable table;
    private NetworkTableEntry shooterSpeedRPS;

    //Change right motor to inverted using documentation
    public static double LEFT_MOTOR_SPEED = 0.25;
    public static double RIGHT_MOTOR_SPEED = LEFT_MOTOR_SPEED;


    CANSparkFlex leftShooterMotor;
    CANSparkFlex rightShooterMotor;
    RelativeEncoder leftShooterEncoder;
    RelativeEncoder rightShooterEncoder;
    XboxController operatorController;
    XboxController driverController;
    SparkPIDController leftShooterControl;
    SparkPIDController rightShooterControl;

    public CrescendoShooter(){
        leftShooterMotor = new CANSparkFlex(LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        rightShooterMotor = new CANSparkFlex(RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
        rightShooterMotor.setInverted(true);
        leftShooterEncoder = leftShooterMotor.getEncoder();
        rightShooterEncoder = rightShooterMotor.getEncoder();

        table = NetworkTableInstance.getDefault().getTable(shooterTableName);
        leftShooterControl =  leftShooterMotor.getPIDController();
        rightShooterControl = rightShooterMotor.getPIDController();
    }

    public void shoot(double speedRPM){
        leftShooterControl.setReference(speedRPM, com.revrobotics.CANSparkBase.ControlType.kVelocity); 
        rightShooterControl.setReference(speedRPM, com.revrobotics.CANSparkBase.ControlType.kVelocity);
    }

    public void stop(){
        leftShooterControl.setReference(0, com.revrobotics.CANSparkBase.ControlType.kVelocity);
        rightShooterControl.setReference(0, com.revrobotics.CANSparkBase.ControlType.kVelocity);
    }
}
