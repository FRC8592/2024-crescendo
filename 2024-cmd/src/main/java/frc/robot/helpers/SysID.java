package frc.robot.helpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class SysID {

    private Measure<Voltage> volts;
    private SparkFlexControl testMotor;
    private String name;
    private SubsystemBase subsystemBase;
    private PIDController pidController;
    private SimpleMotorFeedforward feedforward;
    
    /**
     * Creates an instance of the SysID object which can be used to calculate feedforward, pid values, voltage, and run the necessary
     * SysID tests to help tune PID faster. 
     * 
     * @param testMotor the motor being used for the SysID test
     * @param name the name of the motor for logging purposes
     * @param subsystemBase the subsystem the motor is a part of
     */
    public SysID(SparkFlexControl testMotor, String name, SubsystemBase subsystemBase){
        this.testMotor = testMotor;
        this.name = name;
        this.subsystemBase = subsystemBase;
    }

    /**
     * The voltage assigned turned into WPILib voltage units to use in SysID tests so we can appropriately measure for the application.
     * 
     * @return the voltage in the WPILib units
     */
    public Measure<Voltage> getVoltageDoubleToVoltageUnits(){

        MutableMeasure<Voltage> volt = mutable(Volts.of(testMotor.motor.getBusVoltage()));

        double motorVoltage = volts.baseUnitMagnitude();

        return volt.mut_replace(motorVoltage, Volts);


    }

    /**
     * The current position of the motor turned into WPILib angle units to use in SysID tests so we can appropriately measure for the application.
     *
     * @return the angle of the motor in WPILib units
     */
    public Measure<Angle> getPositionDoubleToPositionUnits(){

        MutableMeasure<Angle> angle = mutable(Rotations.of(0));

        return angle.mut_replace(testMotor.getPosition(), Rotations);


    }

    public Measure<Velocity<Angle>> getVelocityDoubleToVelocityUnits(){

        MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));

        return velocity.mut_replace(testMotor.getVelocity(), RotationsPerSecond);

    }

    public void runMotorAtVoltage(Measure<Voltage> volts){
        this.volts = volts;
        Logger.recordOutput(name + "/Voltage", volts.baseUnitMagnitude());
        Logger.recordOutput(name + "/Position", testMotor.getPosition());
        Logger.recordOutput(name + "/Velocity", testMotor.getVelocity());

        testMotor.motor.setVoltage(volts.baseUnitMagnitude());
    }

    public SysIdRoutine createRoutine(){

        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> {runMotorAtVoltage(voltage);}, 
                (log)->{log.motor(name)
                    .voltage(getVoltageDoubleToVoltageUnits())
                    .angularPosition(getPositionDoubleToPositionUnits())
                    .angularVelocity(getVelocityDoubleToVelocityUnits());
                }, subsystemBase
            )
        );

        return routine;

    }

    public void setPID( double kP, double kI, double kD){
        pidController = new PIDController(kP, kI, kD);
    }

    public void setFeedforward(double kS, double kV, double kA){
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public double calculatedFeedforward(double targetSpeed){
        return feedforward.calculate(targetSpeed);
    }

    public double calculateVoltage(double velocity, double targetSpeed){
        return pidController.calculate(velocity, targetSpeed) + feedforward.calculate(targetSpeed);
    }
}
