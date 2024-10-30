package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.*;
import frc.robot.subsystems.SubsystemCommands;

public class IntakeCommands extends SubsystemCommands{
    private Intake intake;

    public IntakeCommands(Intake intake){
        this.intake = intake;
    }

    /**
     * Command to run the intake motor at the outake velocity constant
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted to end
     */
    public Command outakeCommand() {
        return intake.runEnd(() -> {
            // Run continuously until interrupted
            intake.setIntakeVelocity(INTAKE.OUTAKE_VELOCITY);
        }, () -> {
            // Run once when interrupted
            intake.setIntakeVelocity(0);
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
        return intake.runEnd(() -> {
            intake.setIntakeVelocity(INTAKE.INTAKE_VELOCITY);
        }, () -> {
            intake.setIntakeVelocity(0);
        });
    }

    /**
     * Command to stop the intake roller
     *
     * @return the command
     *
     * @apiNote This command runs instantly and ends on the same frame
     */
    public Command stopCommand(){
        return intake.runOnce(() -> {intake.stop();});
    }

    /**
     * Command to stop the intake
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command autonomousInitCommand(){
        return stopCommand();
    }
}