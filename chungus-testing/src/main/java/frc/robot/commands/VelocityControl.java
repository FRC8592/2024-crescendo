package frc.robot.commands;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

public class VelocityControl {
    // Constants
    public static final int LEFT_MOTOR_CAN_ID = 31;
    public static final int RIGHT_MOTOR_CAN_ID = 29;

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

    public VelocityControl(){
        leftShooterMotor = new CANSparkFlex(LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        leftShooterMotor.setInverted(true);
        rightShooterMotor = new CANSparkFlex(RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
        leftShooterEncoder = leftShooterMotor.getEncoder();
        rightShooterEncoder = rightShooterMotor.getEncoder();

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

    public void spinPower(double power){
        leftShooterMotor.set(Math.min(Math.max(power,-0.5),0.5)); 
        rightShooterMotor.set(Math.min(Math.max(power,-0.5),0.5));
    }

    public void stopPower(){
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
    }
}
