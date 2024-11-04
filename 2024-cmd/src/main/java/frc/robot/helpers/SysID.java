package frc.robot.helpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SHOOTER;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

public class SysID {

    private Measure<Voltage> volts;
    private SparkFlexControl testMotor;
    private TalonFX[] swerveMotors;
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

    public SysID(TalonFX swerveMotor1, TalonFX swerveMotor2, TalonFX swerveMotor3, TalonFX swerveMotor4, String name, SubsystemBase subsystemBase){
        swerveMotors = new TalonFX[4];
        swerveMotors[0] = swerveMotor1;
        swerveMotors[1] = swerveMotor2;
        swerveMotors[2] = swerveMotor3;
        swerveMotors[3] = swerveMotor4;

        this.name = name;
        this.subsystemBase = subsystemBase;
    }

    /**
     * The voltage assigned turned into WPILib voltage units to use in SysID tests so we can appropriately measure for the application.
     * 
     * @return the voltage in the WPILib units
     */
    private Measure<Voltage> getVoltageDoubleToVoltageUnits(){

        MutableMeasure<Voltage> volt = mutable(Volts.of(testMotor.motor.getBusVoltage()));

        double motorVoltage = volts.baseUnitMagnitude();

        return volt.mut_replace(motorVoltage, Volts);


    }

    private Measure<Voltage> getSwerveVoltageDoubleToVoltageUnits(TalonFX swerveMotor){

        MutableMeasure<Voltage> volt = mutable(Volts.of(swerveMotor.getMotorVoltage().getValueAsDouble()));

        double motorVoltage = volts.baseUnitMagnitude();

        return volt.mut_replace(motorVoltage, Volts);


    }

    /**
     * The current position of the motor turned into WPILib angle units to use in SysID tests so we can appropriately measure for the application.
     *
     * @return the angle of the motor in WPILib units
     */
    private Measure<Angle> getPositionDoubleToPositionUnits(){

        MutableMeasure<Angle> angle = mutable(Rotations.of(0));

        return angle.mut_replace(testMotor.getPosition(), Rotations);


    }

    /**
     * The current position of the motor turned into WPILib distance units to use in SysID tests so we can appropriately measure for the application.
     *
     * @return the angle of the motor in WPILib units
     */
    private Measure<Distance> getPositionDoubleToPositionUnitsLinear(TalonFX swerveMotor){

        MutableMeasure<Distance> distance = mutable(Meters.of(0));

        return distance.mut_replace(swerveMotor.getPosition().getValueAsDouble(), Meters);


    }

    private Measure<Velocity<Angle>> getVelocityDoubleToVelocityUnits(){

        MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));

        return velocity.mut_replace(testMotor.getVelocity(), RotationsPerSecond);

    }

    private Measure<Velocity<Distance>> getVelocityDoubleToVelocityUnitsLinear(TalonFX swerveMotor){

        MutableMeasure<Velocity<Distance>> velocity = mutable(MetersPerSecond.of(0));

        return velocity.mut_replace(swerveMotor.getVelocity().getValueAsDouble(), MetersPerSecond);

    }

    private void runMotorAtVoltage(Measure<Voltage> volts){
        this.volts = volts;
        Logger.recordOutput(name + "/Voltage", volts.baseUnitMagnitude());
        Logger.recordOutput(name + "/Position", testMotor.getPosition());
        Logger.recordOutput(name + "/Velocity", testMotor.getVelocity());

        testMotor.motor.setVoltage(volts.baseUnitMagnitude());
    }

    private void runSwerveMotorAtVoltage(Measure<Voltage> volts, TalonFX swerveMotor, String nameString){
        this.volts = volts;
        Logger.recordOutput(name + "/Voltage", volts.baseUnitMagnitude());
        Logger.recordOutput(name + "/Position", swerveMotor.getPosition().getValueAsDouble());
        Logger.recordOutput(name + "/VelocityRPM", swerveMotor.getVelocity().getValueAsDouble());

        swerveMotor.setVoltage(volts.baseUnitMagnitude());
    }

    private void runSwerveMotorsAtVoltage(Measure<Voltage> volts){
        runSwerveMotorAtVoltage(volts, swerveMotors[0], "frontLeftMotor");
        runSwerveMotorAtVoltage(volts, swerveMotors[1], "frontRightMotor");
        runSwerveMotorAtVoltage(volts, swerveMotors[2], "backLeftMotor");
        runSwerveMotorAtVoltage(volts, swerveMotors[3], "backRightMotor");
    }

    private void logSwerveMotors(SysIdRoutineLog log){
        log.motor("frontLeftMotor")
            .voltage(getSwerveVoltageDoubleToVoltageUnits(swerveMotors[0]))
            .linearPosition(getPositionDoubleToPositionUnitsLinear(swerveMotors[0]))
            .linearVelocity(getVelocityDoubleToVelocityUnitsLinear(swerveMotors[0]));

        log.motor("frontRightMotor")
            .voltage(getSwerveVoltageDoubleToVoltageUnits(swerveMotors[1]))
            .linearPosition(getPositionDoubleToPositionUnitsLinear(swerveMotors[1]))
            .linearVelocity(getVelocityDoubleToVelocityUnitsLinear(swerveMotors[1]));

        log.motor("backLeftMotor")
            .voltage(getSwerveVoltageDoubleToVoltageUnits(swerveMotors[2]))
            .linearPosition(getPositionDoubleToPositionUnitsLinear(swerveMotors[2]))
            .linearVelocity(getVelocityDoubleToVelocityUnitsLinear(swerveMotors[2]));

        log.motor("backRightMotor")
            .voltage(getSwerveVoltageDoubleToVoltageUnits(swerveMotors[3]))
            .linearPosition(getPositionDoubleToPositionUnitsLinear(swerveMotors[3]))
            .linearVelocity(getVelocityDoubleToVelocityUnitsLinear(swerveMotors[3]));
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

    public SysIdRoutine createRoutineSwerve(int timeout){

        MutableMeasure<Time> time = mutable(Seconds.of(timeout)); //Not being used yet, for testing purposes.

        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    runSwerveMotorsAtVoltage(voltage);
                }, 
                (log)->{
                    logSwerveMotors(log);
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
        
        Logger.recordOutput(SHOOTER.LOG_PATH+"VoltageModeStatus", true);

        return pidController.calculate(velocity, targetSpeed) + feedforward.calculate(targetSpeed);
    }
}