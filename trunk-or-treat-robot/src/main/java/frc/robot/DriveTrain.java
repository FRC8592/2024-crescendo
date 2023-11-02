package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
    private WPI_TalonFX frontLeft;
    private WPI_TalonFX backLeft;
    private WPI_TalonFX frontRight;
    private WPI_TalonFX backRight;

    public DriveTrain() {
        frontLeft = new WPI_TalonFX(Constants.LEFT_FRONT_CAN_ID);
        backLeft = new WPI_TalonFX(Constants.LEFT_BACK_CAN_ID);
        frontRight = new WPI_TalonFX(Constants.RIGHT_FRONT_CAN_ID);
        backRight = new WPI_TalonFX(Constants.RIGHT_BACK_CAN_ID);
        frontLeft.configFactoryDefault();
        frontLeft.setInverted(true);
        backLeft.configFactoryDefault();
        backLeft.setInverted(true);
        backLeft.follow(frontLeft);
        frontRight.configFactoryDefault();
        backRight.configFactoryDefault();
        backRight.follow(frontRight);
    }

    public void xyDrive(double stickX, double stickY) {
        double leftMotor = stickY + stickX;
        double rightMotor = stickY - stickX;
        leftMotor = leftMotor * Constants.motorPowerMultiplier;
        rightMotor = rightMotor * Constants.motorPowerMultiplier;
        double motorScale = 1;
        if (Math.abs(leftMotor) > 1) {
            motorScale = 1 / Math.abs(leftMotor);
        }
        if (Math.abs(rightMotor) > 1) {
            motorScale = 1 / Math.abs(rightMotor);
        }
        rightMotor = deadBand(rightMotor * motorScale);
        leftMotor = deadBand(leftMotor * motorScale);
        frontLeft.set(ControlMode.PercentOutput, rightMotor);
        frontRight.set(ControlMode.PercentOutput, leftMotor);
        SmartDashboard.putNumber("Left Side", leftMotor);
        SmartDashboard.putNumber("Right Side", rightMotor);
    }

    public void haltDriveTrain() {
        frontLeft.set(ControlMode.PercentOutput, 0);
        frontRight.set(ControlMode.PercentOutput, 0);
        SmartDashboard.putNumber("Left Side", 0);
        SmartDashboard.putNumber("Right Side", 0);
    }

    public double deadBand(double value) {
        if (Math.abs(value) <= Constants.deadBandValue) {
            return 0;
        } else {
            return value;
        }
    }
}