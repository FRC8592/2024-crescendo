package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
// Pneumatic control classes
// import packages
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    private SparkFlexControl topMotor;
    private SparkFlexControl bottomMotor;

    private enum States {
        INTAKE_PREP,
        INTAKING,
        STOP
    }
    
    private States state;
    

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
        topMotor = new SparkFlexControl(INTAKE.TOP_MOTOR_CAN_ID, true);
        topMotor.setPIDF(INTAKE.TOP_MOTOR_kP, INTAKE.TOP_MOTOR_kI, INTAKE.TOP_MOTOR_kD, 0, 0);
        
        bottomMotor = new SparkFlexControl(INTAKE.BOTTOM_MOTOR_CAN_ID,true);
        bottomMotor.setPIDF(INTAKE.BOTTOM_MOTOR_kP, INTAKE.BOTTOM_MOTOR_kI, INTAKE.BOTTOM_MOTOR_kD, 0, 0);


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
                setIntakeVelocity(INTAKE.SPEED_TOP, INTAKE.SPEED_BOTTOM);
                if (shooter.hasNote()) {
                    this.state = States.STOP;
                }
                break;
            case STOP:
            default:
                setIntakeVelocity(0, 0);
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

    public void halt() {
        topMotor.setPercentOutput(0);
        bottomMotor.setPercentOutput(0);
    }

    /**
     * Spins the intake motors at the given power ({@code PercentOutput})
     * @param speed the power to send to the motors
     */
    public void spinPercentOutput(double speed) {
        topMotor.setPercentOutput(speed);
        bottomMotor.setPercentOutput(speed);
    }

    /**
     * Gets the velocity of the top motor in meters per second
     * @return topMotorVelocityMetersPerSecond
     */
    public double getTopMotorVelocityRPM() {
        double topMotorVelocity = topMotor.getVelocity();
        SmartDashboard.putNumber("Measured Intake Top RPM", topMotorVelocity);
        return topMotorVelocity;
    }

    /**
     * Gets the velocity of the bottom motor in meters per second
     * @return bottomMotorVelocityMetersPerSecond
     */
    public double getBottomMotorVelocityRPM() {
        double bottomMotorVelocity = bottomMotor.getVelocity();
        SmartDashboard.putNumber("Measured Intake Bottom RPM", bottomMotorVelocity);
        return bottomMotorVelocity;
    }

    /**
     * Run the motors with velocity control
     * @param bottom Velocity for bottom motor (RPM)
     * @param top Velocity for top motor (RPM)
     */
    public void setIntakeVelocity(double bottom, double top) {
        topMotor.setVelocity(top);
        bottomMotor.setVelocity(bottom);

        getTopMotorVelocityRPM();
        getBottomMotorVelocityRPM();
    }

    public void setIntakePercentOutput(double bottom, double top){
        topMotor.setPercentOutput(top);
        bottomMotor.setPercentOutput(bottom);
    }
    /**
     * Run the motors proportionally to the drivetrain speed
     * @param swerve Drivetrain object to get speed from
     */
    public void robotSpeedIntake (Swerve swerve) {
        double robotSpeed = swerve.getCurrentSpeeds().vyMetersPerSecond;
        double rollerSpeed = Math.max(INTAKE.MINIMUM_ROLLER_SPEED, robotSpeed * INTAKE.ROBOT_SPEED_MULTIPLIER);
        topMotor.setVelocity(rollerSpeed);
        bottomMotor.setVelocity(rollerSpeed);
    }

    /**TODO:WRITE THIS METHOD PLS 
     * @return if intake has note
     */
    public boolean hasNote() {
        return false;
    }
}
