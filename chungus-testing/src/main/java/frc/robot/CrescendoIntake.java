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
    public static final double TOP_MOTOR_kP = 0;
    public static final double TOP_MOTOR_kI = 0;
    public static final double TOP_MOTOR_kD = 0;
    public static final double BOTTOM_MOTOR_kD = 0;
    public static final double BOTTOM_MOTOR_kP = 0;
    public static final double BOTTOM_MOTOR_kI = 0;
    public static final int TOP_MOTOR_INTAKE_SPEED = 1000;
    public static final int BOTTOM_MOTOR_INTAKE_SPEED = 2000;

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
     * Run the motors with velocity control (CURRENTLY NOT SET)
     */
    public void intakeNote(double bottom, double top) {
        topMotor.set(ControlMode.Velocity, top);
        bottomMotor.set(ControlMode.Velocity, bottom);
    }
}
