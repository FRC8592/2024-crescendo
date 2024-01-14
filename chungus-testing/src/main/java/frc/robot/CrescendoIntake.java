package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

// Pneumatic control classes
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import packages
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CrescendoIntake {
    //Defining constants for this class
    public static final int TOP_MOTOR_CAN_ID = 15;
    public static final int BOTTOM_MOTOR_CAN_ID = 18;
    public static final double TOP_MOTOR_kP = 0.01;
    public static final double TOP_MOTOR_kI = 0.001;
    public static final double TOP_MOTOR_kD = 0.001;
    public static final double BOTTOM_MOTOR_kP = 0.01;
    public static final double BOTTOM_MOTOR_kI = 0.001;
    public static final double BOTTOM_MOTOR_kD = 0.001;
    public static final double BOTTOM_ROLLER_DIAMETER = 0.0254; // meters
    public static final double TOP_ROLLER_DIAMETER = 0.0508;

    private TalonFX topMotor;
    private TalonFX bottomMotor;

    public CrescendoIntake() {
        topMotor = new TalonFX(TOP_MOTOR_CAN_ID);
        topMotor.setInverted(true);
        bottomMotor = new TalonFX(BOTTOM_MOTOR_CAN_ID);

        topMotor.config_kP(0, TOP_MOTOR_kP);
        topMotor.config_kI(0, TOP_MOTOR_kI);
        topMotor.config_kD(0, TOP_MOTOR_kD);
        bottomMotor.config_kP(0, BOTTOM_MOTOR_kP);
        bottomMotor.config_kI(0, BOTTOM_MOTOR_kI);
        bottomMotor.config_kD(0, BOTTOM_MOTOR_kD);
    }

    /**
     * Spins the intake motors at the given power ({@code PercentOutput})
     * @param speed the power to send to the motors
     */
    public void spinPercentOutput(double speed) {
        topMotor.set(ControlMode.PercentOutput, speed);
        bottomMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Gets the velocity of the top motor in meters per second
     * @return topMotorVelocityMetersPerSecond
     */
    public double getTopMotorVelocityRPM() {
        double topMotorVelocity = topMotor.getSelectedSensorVelocity();
        double topMotorVelocityRPM = UnitUtil.ticksToRPM(topMotorVelocity);
        SmartDashboard.putNumber("Top Vel Ticks", topMotorVelocity);
        SmartDashboard.putNumber("Top Motor Velocity (RPM)", topMotorVelocityRPM);
        return topMotorVelocityRPM;
    }

    /**
     * Gets the velocity of the bottom motor in meters per second
     * @return bottomMotorVelocityMetersPerSecond
     */
    public double getBottomMotorVelocityRPM() {
        double bottomMotorVelocity = bottomMotor.getSelectedSensorVelocity();
        double bottomMotorVelocityRPM = UnitUtil.ticksToRPM(bottomMotorVelocity);
        SmartDashboard.putNumber("Bottom Vel Ticks", bottomMotorVelocity);
        SmartDashboard.putNumber("Bottom Motor Velocity (RPM)", bottomMotorVelocityRPM);
        return bottomMotorVelocityRPM;
    }

    /**
     * Run the motors with velocity control
     * @param bottom Velocity for bottom motor (RPM)
     * @param top Velocity for top motor (RPM)
     */
    public void intakeNote(double bottom, double top) {
        double topVelocity = UnitUtil.RPMToTicks(top); // Convert RPM to ticks (100ms)
        double bottomVelocity = UnitUtil.RPMToTicks(bottom); // Convert RPM to ticks (100ms)

        topMotor.set(ControlMode.Velocity, topVelocity);
        bottomMotor.set(ControlMode.Velocity, bottomVelocity);

        getTopMotorVelocityRPM();
        getBottomMotorVelocityRPM();
    }
}
