package frc.robot.subsystems.singlemotor;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SINGLEMOTOR;

public class SingleMotor extends SubsystemBase{
    private static SingleMotor instance = null;
    public static SingleMotor getInstance(){
        if(instance == null){
            throw new IllegalStateException("The SingleMotor subsystem must be instantiated before attempting to use it");
        }
        return instance;
    }
    public static SingleMotor instantiate(){
        if(instance != null){
            throw new IllegalStateException("The SingleMotor subsystem can't be instantiated twice");
        }
        instance = new SingleMotor();
        return instance;
    }
    public SingleMotorCommands commands = new SingleMotorCommands(this);

    private SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((voltage) -> {runMotorAtVoltage(voltage);}, null, this, "singleMotorLogs")
        );

    private TalonFX motor;
    private VelocityDutyCycle motorVelocity = new VelocityDutyCycle(0);

    private SingleMotor(){
        motor = new TalonFX(CAN.MOTOR_CAN_ID);
        TalonFXConfiguration config = new TalonFXConfiguration(); // Contains factory settings

        config.Slot0.kP = SINGLEMOTOR.PID_P;
        config.Slot0.kI = SINGLEMOTOR.PID_I;
        config.Slot0.kD = SINGLEMOTOR.PID_D;
        config.Slot0.kS = SINGLEMOTOR.FF_S;
        config.Slot0.kV = SINGLEMOTOR.FF_V; // Renamed from kF in Phoenix 5
        config.Slot0.kA = SINGLEMOTOR.FF_A;

        motor.getConfigurator().apply(config);
        motor.set(0);

        
    }

    /**
     * Run the motor at a set voltage. This is what the SysID
     * routines should need.
     *
     * @param volts the voltage
     */
    protected void runMotorAtVoltage(Measure<Voltage> volts){
        Logger.recordOutput("SysID/Voltage", volts.baseUnitMagnitude());
        Logger.recordOutput("SysID/Position", motor.getPosition().getValueAsDouble());
        Logger.recordOutput("SysID/Velocity", motor.getVelocity().getValueAsDouble());
        
        motor.setVoltage(volts.baseUnitMagnitude());
    }

    /**
     * Run the motor at a set velocity in RPS.
     *
     * @param velocityRPS the velocity in RPS
     *
     * @apiNote <i><b>WARNING: THIS IS IN RPS,
     * NOT RPM. Be careful.</b></i>
     */
    protected void runMotorAtVelocity(int velocityRPS){
        motorVelocity.Velocity = velocityRPS;
        motor.setControl(motorVelocity);
    }

    /**
     * Stop the motor
     */
    protected void stopMotor(){
        motor.stopMotor();
    }

    public void periodic(){
        Logger.recordOutput(SINGLEMOTOR.LOG_PATH+"ActualVelocityRPS", motor.getVelocity().getValueAsDouble());
        Logger.recordOutput(SINGLEMOTOR.LOG_PATH+"TargetVelocityRPS", motorVelocity.Velocity);
    }

    public SysIdRoutine getRoutine(){
        return routine;
    }
}