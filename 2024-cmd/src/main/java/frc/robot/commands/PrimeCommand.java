package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.helpers.RangeTable.RangeEntry;
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;

public class PrimeCommand extends WrapperCommand {

    /**
     * Command to do a static prime (see {@link PrimeCommand#PrimeCommand(Supplier, Shooter, Elevator, Intake, LEDs, DoubleSupplier)}
     * for vision-prime). Command never ends on its own; the shoot command logic must cancel this command.
     *
     * @param entry {@code RangeEntry}: the {@code RangeEntry} to prime to
     * @param shooter
     * @param elevator
     * @param intake
     * @param leds
     * @param offsetSupplier {@code DoubleSupplier} lambda that returns the current offset from the target
     * for the honing LEDs
     */
    public PrimeCommand(
        RangeEntry entry,
        Shooter shooter,
        Elevator elevator,
        Intake intake,
        LEDs leds,
        DoubleSupplier offsetSupplier
    ){
        super(
            shooter.commands.primeCommand(entry)
            .alongWith(elevator.commands.setStaticPositionCommand(entry.pivotAngle, entry.elevatorHeight))
            .alongWith(leds.commands.honeCommand(offsetSupplier))
        );
        addRequirements(intake); //We don't actually need this subsystem, but this keeps the intake from doing anything funny while we prime
    }

    /**
     * Command to do a vision prime (see {@link PrimeCommand#PrimeCommand(RangeEntry, Shooter, Elevator, Intake, LEDs, DoubleSupplier)}
     * for static prime). Command never ends on its own; the shoot command logic must cancel this command.
     *
     * @param entry {@code Supplier<RangeEntry>}: lambda that returns the latest update of the {@code RangeEntry}
     * to prime to
     * @param shooter
     * @param elevator
     * @param intake
     * @param leds
     * @param offsetSupplier {@code DoubleSupplier} lambda that returns the current offset from the target
     * for the honing LEDs
     */
    public PrimeCommand(Supplier<RangeEntry> entry, Shooter shooter, Elevator elevator, Intake intake, LEDs leds, DoubleSupplier offsetSupplier){
        super(
            shooter.commands.primeCommand(entry)
            .alongWith(elevator.commands.setUpdatingPositionCommand(() -> entry.get().pivotAngle, () -> entry.get().elevatorHeight))
            .alongWith(leds.commands.honeCommand(offsetSupplier))
        );
        addRequirements(intake); //We don't actually need this subsystem, but this keeps the intake from doing anything funny while we prime
    }
}