package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.helpers.RangeTable;
import frc.robot.subsystems.*;

public class PrimeCommand extends ProxyCommand {
    public PrimeCommand(RangeTable.RangeEntry entry, Shooter shooter, Elevator elevator, Intake intake, LEDs leds, DoubleSupplier offsetSupplier){
        super(
            shooter.shooterPrimeCommand(entry)
            .alongWith(elevator.setStaticPositionCommand(entry.pivotAngle, entry.elevatorHeight))
            .alongWith(leds.honeCommand(offsetSupplier))
        );
        addRequirements(intake); //We don't actually need this subsystem, but this keeps the intake from doing anything funny while we prime
        this.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }


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