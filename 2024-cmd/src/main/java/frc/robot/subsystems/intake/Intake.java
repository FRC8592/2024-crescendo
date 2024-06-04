// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.*;
import frc.robot.subsystems.SubsystemCommands;
import frc.robot.Constants.*;

public class Intake extends SubsystemBase{
    public IntakeCommands commands = new IntakeCommands(this);

    protected SparkFlexControl intakeMotor;
    protected double targetIntakeVelocity = 0;
    public Intake() {
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
}
