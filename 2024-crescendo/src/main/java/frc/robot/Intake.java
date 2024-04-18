package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    public SparkFlexControl topMotor;
    // private SparkFlexControl bottomMotor;

    public Intake() {
        // topMotor = new TalonFX(INTAKE.TOP_MOTOR_CAN_ID);
        // topMotor.setInverted(true);
        // bottomMotor = new TalonFX(INTAKE.BOTTOM_MOTOR_CAN_ID);
// 
        // topMotor.config_kP(0, INTAKE.TOP_MOTOR_kP);
        // topMotor.config_kI(0, INTAKE.TOP_MOTOR_kI);
        // topMotor.config_kD(0, INTAKE.TOP_MOTOR_kD);
        // bottomMotor.config_kP(0, INTAKE.BOTTOM_MOTOR_kP);
        // bottomMotor.config_kI(0, INTAKE.BOTTOM_MOTOR_kI);
        // bottomMotor.config_kD(0, INTAKE.BOTTOM_MOTOR_kD);
        topMotor = new SparkFlexControl(CAN.INTAKE_MOTOR_CAN_ID, true);
        topMotor.setPIDF(INTAKE.MOTOR_kP, INTAKE.MOTOR_kI, INTAKE.MOTOR_kD, INTAKE.MOTOR_kFF, 0);
        topMotor.setCurrentLimit(POWER.INTAKE_MOTOR_CURRENT_LIMIT, POWER.INTAKE_MOTOR_CURRENT_LIMIT);
        // bottomMotor = new SparkFlexControl(INTAKE.BOTTOM_MOTOR_CAN_ID,true);
        // bottomMotor.setPIDF(INTAKE.BOTTOM_MOTOR_kP, INTAKE.BOTTOM_MOTOR_kI, INTAKE.BOTTOM_MOTOR_kD, 0, 0);


    }

    public void stopIntake() {
        topMotor.setPercentOutput(0);
        // bottomMotor.setPercentOutput(0);
    }

    /**
     * Spins the intake motors at the given power ({@code PercentOutput})
     * @param speed the power to send to the motors
     */
    public void spinPercentOutput(double speed) {
        topMotor.setPercentOutput(speed);
        // bottomMotor.setPercentOutput(speed);
    }

    /**
     * Gets the velocity of the top motor in meters per second
     * @return topMotorVelocityMetersPerSecond
     */
    public double getTopMotorVelocityRPM() {
        double topMotorVelocity = topMotor.getVelocity();
        // SmartDashboard.putNumber("Measured Intake Top RPM", topMotorVelocity);
        return topMotorVelocity;
    }

    /**
     * Gets the velocity of the bottom motor in meters per second
     * @return bottomMotorVelocityMetersPerSecond
     */
    /*
    public double getBottomMotorVelocityRPM() {
        double bottomMotorVelocity = bottomMotor.getVelocity();
        SmartDashboard.putNumber("Measured Intake Bottom RPM", bottomMotorVelocity);
        return bottomMotorVelocity;
    }
    */

    /**
     * Run the motors with velocity control
     * @param bottom Velocity for bottom motor (RPM)
     * @param top Velocity for top motor (RPM)
     */
    public void setIntakeVelocity(/*double bottom, */double top) {
        topMotor.setVelocity(top);
        // bottomMotor.setVelocity(bottom);

        getTopMotorVelocityRPM();
        // getBottomMotorVelocityRPM();
    }
}
