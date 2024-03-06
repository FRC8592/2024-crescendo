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

    public Intake() {
        topMotor = new SparkFlexControl(INTAKE.TOP_MOTOR_CAN_ID, true);
        topMotor.setPIDF(INTAKE.MOTOR_kP, INTAKE.MOTOR_kI, INTAKE.MOTOR_kD, INTAKE.MOTOR_kFF, 0);
    }

    public void halt() {
        topMotor.setPercentOutput(0);
    }

    /**
     * Spins the intake motors at the given power ({@code PercentOutput})
     * @param speed the power to send to the motors
     */
    public void setIntakePower(double speed) {
        topMotor.setPercentOutput(speed);
        // bottomMotor.setPercentOutput(speed);
    }

    /**
     * Gets the velocity of the top motor in meters per second
     * @return topMotorVelocityMetersPerSecond
     */
    public double getIntakeVelocity() {
        double topMotorVelocity = topMotor.getVelocity();
        SmartDashboard.putNumber("Measured Intake Top RPM", topMotorVelocity);
        return topMotorVelocity;
    }

    /**
     * Run the motors with velocity control
     * @param bottom Velocity for bottom motor (RPM)
     * @param top Velocity for top motor (RPM)
     */
    public void setIntakeVelocity(/*double bottom, */double top) {
        topMotor.setVelocity(top);

        getIntakeVelocity();
    }
}
