package frc.robot;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    public static double LEFT_MOTOR_kP = 0.0; //PID
    public static double LEFT_MOTOR_kI = 0.0; //PID
    public static double LEFT_MOTOR_kD = 0.0; //PID
    public static double RIGHT_MOTOR_kP = 0.0; //PID
    public static double RIGHT_MOTOR_kI = 0.0; //PID
    public static double RIGHT_MOTOR_kD = 0.0; //PID


    CANSparkFlex leftShooterMotor;
    CANSparkFlex rightShooterMotor;
    RelativeEncoder leftShooterEncoder;
    RelativeEncoder rightShooterEncoder;
    SparkPIDController leftShooterControl;
    SparkPIDController rightShooterControl;

    public CrescendoShooter(){
        leftShooterMotor = new CANSparkFlex(LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        leftShooterMotor.setInverted(true);
        rightShooterMotor = new CANSparkFlex(RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
        leftShooterEncoder = leftShooterMotor.getEncoder();
        rightShooterEncoder = rightShooterMotor.getEncoder();

        table = NetworkTableInstance.getDefault().getTable(shooterTableName);
        leftShooterControl =  leftShooterMotor.getPIDController();
        rightShooterControl = rightShooterMotor.getPIDController();
        leftShooterControl.setP(LEFT_MOTOR_kP); //PID
        leftShooterControl.setI(LEFT_MOTOR_kI); //PID
        leftShooterControl.setD(LEFT_MOTOR_kD); //PID
        rightShooterControl.setP(RIGHT_MOTOR_kP); //PID
        rightShooterControl.setI(RIGHT_MOTOR_kI); //PID
        rightShooterControl.setD(RIGHT_MOTOR_kD); //PID

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
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
    }
}