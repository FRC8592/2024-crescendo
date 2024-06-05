package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
     * @param pivotDegrees {@code double}: the pivot target in degrees
     * @param extensionMeters {@code double}: the extension target in meters
     *
     * @return the command
     *
     * @apiNote This command runs until {@link Elevator#isAtTargetPosition()} returns {@code true}
     */
    public Command setStaticPositionCommand(double pivotDegrees, double extensionMeters){
        return elevator.run(() -> {
            elevator.targetPivot = pivotDegrees;
            elevator.targetExtension = extensionMeters;
            elevator.runElevator();
        }).until(() -> elevator.isAtTargetPosition());
    }

    /**
     * Command to put the elevator at the given position. Ends when the elevator is at the target.
     *
     * @param position {@code Positions}: the position to go to
     *
     * @return the command
     *
     * @apiNote This command runs until {@link Elevator#isAtTargetPosition()} returns {@code true}
     */
    public Command setStaticPositionCommand(Positions position){
        return elevator.run(() -> {
            elevator.targetPivot = position.pivot;
            elevator.targetExtension = position.extension;
            elevator.runElevator();
        }).until(() -> elevator.isAtTargetPosition());
    }

    /**
     * Command to constantly drive the elevator towards a setpoint that can change as desired.
     *
     * @param pivotDegrees {@code DoubleSupplier}: a lambda that returns where the pivot should be
     * @param extensionMeters {@code DoubleSupplier}: a lambda that returns how far the extension
     * should go
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command setUpdatingPositionCommand(DoubleSupplier pivotDegrees, DoubleSupplier extensionMeters){
        return elevator.run(() -> {
            elevator.targetPivot = pivotDegrees.getAsDouble();
            elevator.targetExtension = extensionMeters.getAsDouble();
            elevator.runElevator();
        });
    }

    /**
     * The same as {@link Elevator#setStaticPositionCommand(double, double)}, but allows the setpoint
     * to be edited  on the fly and won't end on its own. This is meant to be used together with 
     * {@link Elevator#incrementElevatorPositionCommand(double, double)}.
     *
     * @param pivotDegrees {@code double}: the pivot setpoint to start with
     * @param extensionMeters {@code double}: the extension setpoint to start with
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command setMalleablePositionCommand(double pivotDegrees, double extensionMeters){
        elevator.targetPivot = pivotDegrees;
        elevator.targetExtension = extensionMeters;
        return elevator.run(() -> {
            elevator.runElevator();
        });
    }

    /**
     * The same as {@link Elevator#setStaticPositionCommand(Position)}, but allows the setpoint
     * to be edited  on the fly and won't end on its own. This is meant to be used together with 
     * {@link Elevator#incrementElevatorPositionCommand(double, double)}.
     *
     * @param position {@code Positions}: the position to go to
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public Command setMalleablePositionCommand(Positions position){
        elevator.targetPivot = position.pivot;
        elevator.targetExtension = position.extension;
        return elevator.run(() -> {
            elevator.runElevator();
        });
    }

    /**
     * If a {@link Elevator#setMalleablePositionCommand(double, double)} command is running, change
     * the pivot and extension setpoints for the elevator to target. Otherwise, does nothing functional.
     *
     * @param pivotDegrees {@code double}: the amount to change the pivot target by each frame
     * @param extensionMeters {@code double}: the amount to change the extension setpoint by each frame
     *
     * @return the command
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     * @apiNote This command does NOT require the Elevator subsystem
     */
    public Command incrementElevatorPositionCommand(double pivotDegrees, double extensionMeters){
        return Commands.run(() -> {
            elevator.targetPivot += pivotDegrees;
            elevator.targetExtension += extensionMeters;
        });
    }
}
