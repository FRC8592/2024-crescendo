package frc.robot.subsystems.singlemotor;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SubsystemCommands;

public class SingleMotorCommands extends SubsystemCommands{
    private SingleMotor singleMotor;
    public SingleMotorCommands(SingleMotor singleMotor){
        this.singleMotor = singleMotor;
    }

    /**
     * Command to to run the motor at a variable target velocity
     *
     * @param velocityRPM a lambda that returns the target velocity in RPM
     *
     * @return the command
     *
     * @apiNote this command never ends on its own; it must be interrupted
     * to end.
     * @apiNote for a fixed target velocity, use
     * {@link SingleMotorCommands#driveMotorWithVelocityCommand(int)}.
     */
    public Command driveMotorWithVelocityCommand(IntSupplier velocityRPM){
        return singleMotor.run(() -> {
            singleMotor.runMotorAtVelocity((velocityRPM.getAsInt())/60);
        });
    }

    /**
     * Command to try to run the motor at a set target velocity
     *
     * @param velocityRPM the target velocity in RPM
     *
     * @return the command
     *
     * @apiNote this command never ends on its own; it must be interrupted
     * to end.
     */
    public Command driveMotorWithVelocityCommand(int velocityRPM){
        return driveMotorWithVelocityCommand(() -> velocityRPM);
    }

    /**
     * Command to cut power to the motor hub
     *
     * @return the command
     *
     * @apiNote this command runs once and ends instantly.
     */
    public Command stopMotorCommand(){
        return singleMotor.runOnce(() -> singleMotor.stopMotor());
    }

    // TODO: Add SysID identification commands here
}
