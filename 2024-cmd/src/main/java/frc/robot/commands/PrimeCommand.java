package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.helpers.RangeTable.RangeEntry;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;

public class PrimeCommand extends NewtonCommand {

    /**
     * Command to do a static prime (see {@link PrimeCommand#PrimeCommand(Supplier, Shooter, Elevator, Intake, LEDs, DoubleSupplier)}
     * for vision-prime).
     *
     * @param entry the {@code RangeEntry} to prime to
     * @param shooter
     * @param elevator
     * @param intake
     * @param leds
     * @param offsetSupplier lambda that returns the current offset from the target for the honing LEDs
     *
     * @apiNote This command does not end on its own; it must be interrupted to stop
     */
    public PrimeCommand(RangeEntry entry, DoubleSupplier offsetSupplier){
        super(
            // Spins up the shooter flywheels
            shooter.commands.primeCommand(entry)

            // Gets the elevator in position
            .alongWith(elevator.commands.setStaticPositionCommand(entry.pivotAngle, entry.elevatorHeight))

            // Runs the honing lights (which indicate to the drivers how far off their target they are)
            .alongWith(leds.commands.honeCommand(offsetSupplier))
        );
        addRequirements(intake); //We don't actually need this subsystem, but this keeps the intake from doing anything funny while we prime
    }

    /**
     * Command to do a vision prime (see {@link PrimeCommand#PrimeCommand(RangeEntry, Shooter, Elevator, Intake, LEDs, DoubleSupplier)}
     * for static prime).
     *
     * @param entry lambda that returns the latest update of the {@code RangeEntry} to prime to
     * @param offsetSupplier {@code DoubleSupplier} lambda that returns the current offset from the target
     * for the honing LEDs
     *
     * @apiNote This command does not end on its own; it must be interrupted to stop
     */
    public PrimeCommand(Supplier<RangeEntry> entry, DoubleSupplier offsetSupplier){
        super(
            // The shooter's primeCommand is overloaded; the version that
            // takes in a Supplier will automatically update the flywheel
            // target speeds as the RangeEntry returned by the Supplier
            // changes.
            shooter.commands.primeCommand(entry)

            // This is similar to the primeCommand; the elevator will update
            // itself with the supplier.
            .alongWith(elevator.commands.setUpdatingPositionCommand(() -> entry.get().pivotAngle, () -> entry.get().elevatorHeight))

            // Runs the honing lights (which indicate to the drivers how far off their target they are)
            .alongWith(leds.commands.honeCommand(offsetSupplier))
        );
        addRequirements(intake); //We don't actually need this subsystem, but this keeps the intake from doing anything funny while we prime
    }
}