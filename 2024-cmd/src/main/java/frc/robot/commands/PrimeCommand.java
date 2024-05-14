package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.helpers.RangeTable;
import frc.robot.subsystems.*;

public class PrimeCommand extends ProxyCommand {

    /**
     * Command to do a static prime (see the other constructor for this class for vision-prime). Command
     * ends when both the shooter and elevator are fully ready to shoot.
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
        RangeTable.RangeEntry entry,
        Shooter shooter,
        Elevator elevator,
        Intake intake,
        LEDs leds,
        DoubleSupplier offsetSupplier
    ){
        super(
            shooter.shooterPrimeCommand(entry)
            .alongWith(elevator.setStaticPositionCommand(entry.pivotAngle, entry.elevatorHeight))
            .deadlineWith(leds.honeCommand(offsetSupplier))
        );
        addRequirements(intake); //We don't actually need this subsystem, but this keeps the intake from doing anything funny while we prime
        this.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    /**
     * Command to do a vision prime (see the other constructor for this class for static prime). Command
     * never ends on its own (the task of checking whether the robot falls on code outside this command).
     * This command cancels itself if a command requiring one of the subsystems is scheduled.
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
    public PrimeCommand(Supplier<RangeTable.RangeEntry> entry, Shooter shooter, Elevator elevator, Intake intake, LEDs leds, DoubleSupplier offsetSupplier){
        super(
            shooter.shooterPrimeCommand(entry)
            .alongWith(elevator.setUpdatingPositionCommand(() -> entry.get().pivotAngle, () -> entry.get().elevatorHeight))
            .alongWith(leds.honeCommand(offsetSupplier))
        );
        addRequirements(intake); //We don't actually need this subsystem, but this keeps the intake from doing anything funny while we prime
        this.withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
}