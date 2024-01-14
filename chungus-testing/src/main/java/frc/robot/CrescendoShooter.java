package frc.robot;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.XboxController;

public class CrescendoShooter {
    // Constants
    public static final int LEFT_MOTOR_CAN_ID = 31;
    public static final int RIGHT_MOTOR_CAN_ID = 29;

    //Change right motor to inverted using documentation
    public static double LEFT_MOTOR_SPEED = 0.25;
    public static double RIGHT_MOTOR_SPEED = LEFT_MOTOR_SPEED;


    CANSparkFlex leftShooterMotor;
    CANSparkFlex rightShooterMotor;
    RelativeEncoder leftShooterEncoder;
    RelativeEncoder rightShooterEncoder;
    XboxController operatorController;
    XboxController driverController;

    public CrescendoShooter(){
        leftShooterMotor = new CANSparkFlex(LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        rightShooterMotor = new CANSparkFlex(RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
        rightShooterMotor.follow(leftShooterMotor, true);
        leftShooterEncoder = leftShooterMotor.getEncoder();
        rightShooterEncoder = rightShooterMotor.getEncoder();
    }

    public void shoot(){
        leftShooterMotor.set(LEFT_MOTOR_SPEED);
        rightShooterMotor.set(RIGHT_MOTOR_SPEED);
    }
}
