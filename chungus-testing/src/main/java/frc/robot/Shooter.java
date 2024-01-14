package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;


public class Shooter{
    private WPI_TalonFX backMotor;
    private WPI_TalonFX frontMotor;
    

    private final String LOG_PATH = "Dead Axle Pop";

    public Shooter(){
        backMotor = new WPI_TalonFX(Constants.DAP_BACK_MOTOR_CAN_ID);
        frontMotor = new WPI_TalonFX(Constants.DAP_FRONT_MOTOR_CAN_ID);

        backMotor.configFactoryDefault();
        frontMotor.configFactoryDefault();

        frontMotor.setInverted(true);

        frontMotor.setNeutralMode(NeutralMode.Brake);
        backMotor.setNeutralMode(NeutralMode.Brake);

        frontMotor.config_kP(0, Constants.DAP_FRONT_MOTOR_kP);
        frontMotor.config_kI(0, Constants.DAP_FRONT_MOTOR_kI);
        frontMotor.config_kD(0, Constants.DAP_FRONT_MOTOR_kD);
        frontMotor.config_kF(0, Constants.DAP_FRONT_MOTOR_kF);
        
        backMotor.config_kP(0, Constants.DAP_BACK_MOTOR_kP);
        backMotor.config_kI(0, Constants.DAP_BACK_MOTOR_kI);
        backMotor.config_kD(0, Constants.DAP_BACK_MOTOR_kD);
        backMotor.config_kF(0, Constants.DAP_BACK_MOTOR_kF);

        frontMotor.selectProfileSlot(0, 0);
        backMotor.selectProfileSlot(0, 0);
    }

    // public void setMotorSpeedBottom(double velocity){
    //     bottomMotor.set(ControlMode.Velocity, UnitUtil.metersPerSecondToTicks(velocity));
    // }

    // public void setMotorSpeedTop(double velocity){
    //     topMotor.set(ControlMode.Velocity, UnitUtil.metersPerSecondToTicks(velocity));
    // }

    // public boolean spin(){
    //     frontMotor.set(ControlMode.Velocity, Constants.DAP_FRONT_VELOCITY_TARGET);
    //     backMotor.set(ControlMode.Velocity, Constants.DAP_FRONT_VELOCITY_TARGET);

    //     double topMotorVelocity = frontMotor.getSelectedSensorVelocity();
    //     double bottomMotorVelocity = backMotor.getSelectedSensorVelocity();

    //     if (Math.abs(Constants.DAP_FRONT_VELOCITY_TARGET - topMotorVelocity) <= Constants.DAP_VELOCITY_TOLERANCE || Math.abs(Constants.DAP_BOTTOM_DELOCITY_TARGET - bottomMotorVelocity) <= Constants.DAP_VELOCITY_TOLERANCE){
    //         return true;
    //     }
    //     else{
    //         return false;
    //     }
    // }

    // public void shoot(Intake DAPIntake){
    //     DAPIntake.spinRoller(Constants.DAR_INTAKE_MOVEMENT_POWER_FOR_SHOOT);
    // }

    public void shoot(Intake DAPIntake) {
        // spin the intake, front, and back motors
        frontMotor.set(ControlMode.Velocity, Constants.FRONT_SHOOTER_SPEED_TICKS_100MS);
        backMotor.set(ControlMode.Velocity, Constants.BACK_SHOOTER_SPEED_TICKS_100MS);
        DAPIntake.spinIntakePercentOutput(Constants.DAP_INTAKE_MOVEMENT_POWER_FOR_SHOOT);
    }
    public void log2(){
        
        Logger.getInstance().recordOutput(LOG_PATH+"/Front Velocity", frontMotor.getSelectedSensorVelocity());
        Logger.getInstance().recordOutput(LOG_PATH+"/Back Velocity", backMotor.getSelectedSensorVelocity());
    }

    public void stop() {
        frontMotor.set(ControlMode.PercentOutput, 0.0);
        backMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void logOutputVelo(){
        SmartDashboard.putNumber("FRONT SHOOTER VELOCITY", frontMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("BACK SHOOTER VELOCITY", backMotor.getSelectedSensorVelocity());
    }

    public void shootWithVelo(double frontTicksPer100MS, double backTicksPer100MS, Intake DAPIntake){
        frontMotor.set(ControlMode.Velocity, frontTicksPer100MS);
        backMotor.set(ControlMode.Velocity, backTicksPer100MS);
        // DAPIntake.spinIntakePercentOutput(Constants.DAP_INTAKE_MOVEMENT_POWER_FOR_SHOOT);

    }
    
    
    
    public void spinFrontMotorPct(double speed){
        frontMotor.set(ControlMode.PercentOutput, speed);
    }
    public void TEST_spinBackMotor(double speed){
        backMotor.set(ControlMode.PercentOutput, speed);
    }
}