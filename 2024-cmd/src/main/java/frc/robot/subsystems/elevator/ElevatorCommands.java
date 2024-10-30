package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ELEVATOR;
import frc.robot.subsystems.SubsystemCommands;
import frc.robot.subsystems.elevator.Elevator.Positions;

public class ElevatorCommands extends SubsystemCommands{
    private Elevator elevator;
    public ElevatorCommands(Elevator elevator){
        this.elevator = elevator;
    }

    /**
     * Command to put the elevator at the given setpoint. Ends when the elevator is at the target.
     *
     * @param pivotDegrees the pivot target in degrees
     * @param extensionMeters the extension target in meters
     *
     * @return the command
     *
     * @apiNote This command runs until {@link Elevator#isAtTargetPosition()} returns {@code true}
     */
    public Command setStaticPositionCommand(double pivotDegrees, double extensionMeters){
        return elevator.run(() -> {
            // Drive the elevator while overwriting the setpoints every frame so nothing can change them
            elevator.setUserDesiredTargetPivot(pivotDegrees);
            elevator.setUserDesiredTargetExtension(extensionMeters);
            elevator.runElevator();
        }).until(() -> elevator.isAtTargetPosition() && elevator.isTargeting(pivotDegrees, extensionMeters)) // Run the elevator until it's at its target.
        .finallyDo(() -> {elevator.stop(); Logger.recordOutput(ELEVATOR.LOG_PATH+"StaticPositionCommandRunning", false);}); // When the command ends, stop the elevator
    }

    /**
     * Command to put the elevator at the given position. Ends when the elevator is at the target.
     *
     * @param position the position to go to
     *
     * @return the command
     *
     * @apiNote This command runs until {@link Elevator#isAtTargetPosition()} returns {@code true}
     */
    public Command setStaticPositionCommand(Positions position){
        return setStaticPositionCommand(position.pivot, position.extension);
    }

    /**
     * Command to constantly drive the elevator towards a setpoint that can change as desired.
     *
     * @param pivotDegrees a lambda that returns where the pivot should be
     * @param extensionMeters a lambda that returns how far the extension should go
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command setUpdatingPositionCommand(DoubleSupplier pivotDegrees, DoubleSupplier extensionMeters){
        return elevator.run(() -> {
            elevator.setUserDesiredTargetPivot(pivotDegrees.getAsDouble());
            elevator.setUserDesiredTargetExtension(extensionMeters.getAsDouble());
            elevator.runElevator();
        }).finallyDo(() -> elevator.stop());
    }

    /**
     * The same as {@link ElevatorCommands#setStaticPositionCommand(double, double)}, but allows the setpoint
     * to be edited on the fly and won't end on its own. This is meant to be used together with 
     * {@link ElevatorCommands#incrementElevatorPositionCommand(double, double)}.
     *
     * @param pivotDegrees the pivot setpoint to start with
     * @param extensionMeters the extension setpoint to start with
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command setMalleablePositionCommand(double pivotDegrees, double extensionMeters){
        return elevator.runOnce(() -> {
            elevator.setUserDesiredTargetPivot(pivotDegrees);
            elevator.setUserDesiredTargetExtension(extensionMeters);
        }).andThen(elevator.run(() -> {
            elevator.runElevator();
        })).finallyDo(() -> elevator.stop());
    }

    /**
     * The same as {@link ElevatorCommands#setStaticPositionCommand(Position)}, but allows the setpoint
     * to be edited  on the fly and won't end on its own. This is meant to be used together with 
     * {@link ElevatorCommands#incrementElevatorPositionCommand(double, double)}.
     *
     * @param position the position to go to
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command setMalleablePositionCommand(Positions position){
        return setMalleablePositionCommand(position.pivot, position.extension);
    }

    /**
     * If a {@link ElevatorCommands#setMalleablePositionCommand(double, double)} command is running, change
     * the pivot and extension setpoints for the elevator to target. Otherwise, does nothing functional.
     *
     * @param pivotDegrees the amount to change the pivot target by each frame
     * @param extensionMeters the amount to change the extension setpoint by each frame
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     * @apiNote This command does NOT require the Elevator subsystem
     */
    public Command incrementElevatorPositionCommand(double pivotDegrees, double extensionMeters){
        return Commands.run(() -> {
            elevator.setUserDesiredTargetPivot(elevator.getUserDesiredTargetPivot()+pivotDegrees);
            elevator.setUserDesiredTargetExtension(elevator.getUserDesiredTargetExtension()+extensionMeters);
        });
    }

    /**
     * Command to freeze the elevator wherever it is
     *
     * @return the command
     *
     * @apiNote This command runs instantly and ends on the same frame
     */
    public Command stopCommand(){
        return elevator.runOnce(() -> elevator.stop());
    }
}
