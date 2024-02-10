package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
// Pneumatic control classes
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import packages
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    private TalonFX topMotor;
    private TalonFX bottomMotor;

    private enum States {
        INTAKE_PREP,
        INTAKING,
        STOP
    }
    
    private States state;

    public Intake() {
        topMotor = new TalonFX(INTAKE.TOP_MOTOR_CAN_ID);
        topMotor.setInverted(true);
        bottomMotor = new TalonFX(INTAKE.BOTTOM_MOTOR_CAN_ID);

        topMotor.config_kP(0, INTAKE.TOP_MOTOR_kP);
        topMotor.config_kI(0, INTAKE.TOP_MOTOR_kI);
        topMotor.config_kD(0, INTAKE.TOP_MOTOR_kD);
        bottomMotor.config_kP(0, INTAKE.BOTTOM_MOTOR_kP);
        bottomMotor.config_kI(0, INTAKE.BOTTOM_MOTOR_kI);
        bottomMotor.config_kD(0, INTAKE.BOTTOM_MOTOR_kD);
    }

    public void update(Swerve swerve, Shooter shooter) {
        switch (state) {
            case INTAKE_PREP:
                robotSpeedIntake(swerve);
                if (this.hasNote()) {
                    this.state = States.INTAKING;
                }
                break;
            case INTAKING:
                intakeNote(INTAKE.SPEED_TOP, INTAKE.SPEED_BOTTOM);
                if (shooter.hasNote()) {
                    this.state = States.STOP;
                }
                break;
            case STOP:
            default:
                intakeNote(0, 0);
                break;
        }
    }

    public void intake() {
        if (state == States.STOP) {
            this.state = States.INTAKE_PREP;
        }
    }

    public void stop() {
        this.state = States.STOP;
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
    /**
     * Run the motors proportionally to the drivetrain speed
     * @param swerve Drivetrain object to get speed from
     */
    public void robotSpeedIntake (Swerve swerve) {
        double robotSpeed = swerve.getCurrentSpeeds().vyMetersPerSecond;
        double rollerSpeed = Math.max(INTAKE.MINIMUM_ROLLER_SPEED, robotSpeed * INTAKE.ROBOT_SPEED_MULTIPLIER);
        topMotor.set(ControlMode.Velocity, rollerSpeed );
        bottomMotor.set(ControlMode.Velocity, rollerSpeed);

    }

    /**TODO:WRITE THIS METHOD PLS 
     * @return if intake has note
     */
    public boolean hasNote() {
        return false;
    }
}
