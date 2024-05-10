package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.helpers.RangeTable;
import frc.robot.subsystems.*;

public class PrimeCommand extends ProxyCommand {
    Supplier<RangeTable.RangeEntry> entrySupplier;

    public PrimeCommand(Supplier<RangeTable.RangeEntry> entry, Shooter shooter, Elevator elevator, Intake intake){
        super(
            shooter.shooterPrimeCommand(entry)
            .alongWith(elevator.setElevatorPositionCommand(entry))
        );
        this.withInterruptBehavior(InterruptionBehavior.kCancelSelf);
        addRequirements(intake); //We don't actually need this subsystem, but this keeps the intake from doing anything funny while we prime
    }
    public PrimeCommand(DoubleSupplier distance, Shooter shooter, Elevator elevator, Intake intake){
        this(() -> RangeTable.get(distance.getAsDouble()), shooter, elevator, intake);
    }

    public PrimeCommand(RangeTable.RangeEntry entry, Shooter shooter, Elevator elevator, Intake intake){
        super(
            shooter.shooterPrimeCommand(entry)
            .alongWith(elevator.setElevatorPositionCommand(entry))
        );
        this.withInterruptBehavior(InterruptionBehavior.kCancelSelf);
        addRequirements(intake); //We don't actually need this subsystem, but this keeps the intake from doing anything funny while we prime
    }
    public PrimeCommand(double distance, Shooter shooter, Elevator elevator, Intake intake){
        this(() -> RangeTable.get(distance), shooter, elevator, intake);
    }
}