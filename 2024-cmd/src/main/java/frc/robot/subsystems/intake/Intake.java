// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.NewtonSubsystem;

import frc.robot.helpers.*;
import frc.robot.Constants.*;

public class Intake extends NewtonSubsystem{
    private static Intake instance = null;
    public static Intake getInstance(){
        if(instance == null){
            throw new IllegalStateException("The Intake subsystem must be instantiated before attempting to use it");
        }
        return instance;
    }
    public static Intake instantiate(){
        if(instance != null){
            throw new IllegalStateException("The Intake subsystem can't be instantiated twice");
        }
        instance = new Intake();
        return instance;
    }

    public IntakeCommands commands = new IntakeCommands(this);

    private SparkFlexControl intakeMotor;

    // Used for logging
    private double targetIntakeVelocity = 0;

    private Intake() {
        intakeMotor = new SparkFlexControl(CAN.INTAKE_MOTOR_CAN_ID, true);
        intakeMotor.setPIDF(INTAKE.MOTOR_kP, INTAKE.MOTOR_kI, INTAKE.MOTOR_kD, INTAKE.MOTOR_kFF, 0);
        intakeMotor.setCurrentLimit(POWER.INTAKE_MOTOR_CURRENT_LIMIT, POWER.INTAKE_MOTOR_CURRENT_LIMIT);
    }

    public void periodic() {
        Logger.recordOutput(INTAKE.LOG_PATH+"IntakeVelocityRPM", intakeMotor.getVelocity());
        Logger.recordOutput(INTAKE.LOG_PATH+"IntakeTargetRPM", targetIntakeVelocity);
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * Set the intake motor's velocity
     * @param velocity the target velocity in RPM
     */
    protected void setIntakeVelocity(double velocity){
        targetIntakeVelocity = velocity;
        intakeMotor.setVelocity(velocity);
    }

    protected void stop(){
        targetIntakeVelocity = 0;
        setIntakeVelocity(0);
    }
}
