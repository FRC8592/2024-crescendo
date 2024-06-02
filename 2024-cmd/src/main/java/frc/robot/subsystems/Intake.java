// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.*;
import frc.robot.Constants.*;

public class Intake extends NewtonSubsystem {
    private SparkFlexControl intakeMotor;
    private double targetIntakeVelocity = 0;
    public Intake() {
        intakeMotor = new SparkFlexControl(CAN.INTAKE_MOTOR_CAN_ID, true);
        intakeMotor.setPIDF(INTAKE.MOTOR_kP, INTAKE.MOTOR_kI, INTAKE.MOTOR_kD, INTAKE.MOTOR_kFF, 0);
        intakeMotor.setCurrentLimit(POWER.INTAKE_MOTOR_CURRENT_LIMIT, POWER.INTAKE_MOTOR_CURRENT_LIMIT);
    }

    /**
     * Command to run the intake motor at the outake velocity constant
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted to end
     */
    public Command outakeCommand() {
        return runEnd(() -> {
            // Run continuously until interrupted
            targetIntakeVelocity = INTAKE.OUTAKE_VELOCITY;
            intakeMotor.setVelocity(INTAKE.OUTAKE_VELOCITY);
        }, () -> {
            // Run once when interrupted
            targetIntakeVelocity = 0;
            intakeMotor.setVelocity(0);
        });
    }

    /**
     * Command to run the intake motor at the intake velocity constant
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted to end
     */
    public Command intakeCommand(){
        return runEnd(() -> {
            targetIntakeVelocity = INTAKE.INTAKE_VELOCITY;
            intakeMotor.setVelocity(INTAKE.INTAKE_VELOCITY);
        }, () -> {
            targetIntakeVelocity = 0;
            intakeMotor.setVelocity(0);
        });
    }

    /**
     * Command to stop the intake
     *
     * @return the command
     *
     * @apiNote This command runs instantly and ends on the same frame
     */
    public Command stopCommand(){
        return runOnce(() -> {
            targetIntakeVelocity = 0;
            intakeMotor.setVelocity(0);
        });
    }

    public Command autonomousInitCommand(){
        return stopCommand();
    }

    public void periodic() {
        Logger.recordOutput(INTAKE.LOG_PATH+"IntakeVelocityRPM", intakeMotor.getVelocity());
        Logger.recordOutput(INTAKE.LOG_PATH+"IntakeTargetRPM", targetIntakeVelocity);
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
