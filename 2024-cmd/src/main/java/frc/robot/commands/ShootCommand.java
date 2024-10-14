package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.Constants.*;
import frc.robot.commands.proxies.NewtonCommand;
import frc.robot.helpers.RangeTable.RangeEntry;

public class ShootCommand extends NewtonCommand {
    /**
     * Command that dymanically primes to a {@code Supplier<RangeEntry>} and shoots
     * the note when ready. Does not check that the robot is loaded.
     *
     * @param entrySupplier a lambda that returns the most up-to-date {@code RangeEntry}
     * to prime to before shooting
     * @param readyToShoot lambda that returns whether it's time to shoot. Note that
     * this command will never shoot before the shooter and elevator are ready; this
     * argument allows an extra condition (for example, a button) to be added.
     * @param offsetSupplier a {@code DoubleSupplier} that returns the left-right
     * offset from the speaker April tag (used for the honing LEDs).
     *
     * @apiNote This command doesn't have a simple end condition; see the code for
     * details.
     */
    public ShootCommand(Supplier<RangeEntry> entrySupplier, BooleanSupplier readyToShoot, DoubleSupplier offsetSupplier){
        super(
            // Start by priming the robot
            new PrimeCommand(entrySupplier, offsetSupplier).until(() -> (
                readyToShoot.getAsBoolean()
                && shooter.readyToShoot()
                && elevator.isAtTargetPosition()
            )).andThen(
                // Runs the feeder motors
                shooter.commands.fireCommand()
                .alongWith(leds.commands.singleColorCommand(LEDS.OFF))
            )
        );

        // Don't allow the intake to be commanded to do anything funny
        // while shooting
        addRequirements(intake);
    }
    /**
     * Command that dymanically primes to a {@code Supplier<RangeEntry>} and shoots
     * the note when ready. Does not check that the robot is loaded.
     *
     * @param entrySupplier a lambda that returns the most up-to-date {@code RangeEntry}
     * to prime to before shooting
     * @param offsetSupplier a {@code DoubleSupplier} that returns the left-right
     * offset from the speaker April tag (used for the honing LEDs).
     *
     * @apiNote This command doesn't have a simple end condition; see the code for
     * details.
     */
    public ShootCommand(Supplier<RangeEntry> entrySupplier, DoubleSupplier offsetSupplier){
        this(entrySupplier, () -> true, offsetSupplier);
    }

    /**
     * Command that static-primes to a {@code RangeEntry} and shoots the note when ready.
     * Does not check that the robot is loaded.
     *
     * @param entry the {@code RangeEntry} to prime to before shooting
     * @param readyToShoot lambda that returns whether it's time to shoot. Note that
     * this command will never shoot before the shooter and elevator are ready; this
     * argument allows an extra condition (for example, a button) to be added.
     * @param offsetSupplier a {@code DoubleSupplier} that returns the left-right
     * offset from the speaker April tag (used for the honing LEDs).
     *
     * @apiNote This command doesn't have a simple end condition; see the code for
     * details.
     */
    public ShootCommand(RangeEntry entry, BooleanSupplier readyToShoot, DoubleSupplier offsetSupplier){
        this(() -> entry, readyToShoot, offsetSupplier);
    }

    /**
     * Command that static-primes to a {@code RangeEntry} and shoots the note when ready.
     * Does not check that the robot is loaded.
     *
     * @param entry the {@code RangeEntry} to prime to before shooting
     * @param offsetSupplier a {@code DoubleSupplier} that returns the left-right
     * offset from the speaker April tag (used for the honing LEDs).
     *
     * @apiNote This command doesn't have a simple end condition; see the code for
     * details.
     */
    public ShootCommand(RangeEntry entry, DoubleSupplier offsetSupplier){
        this(() -> entry, () -> true, offsetSupplier);
    }

    /**
     * Command that dymanically primes to a {@code Supplier<RangeEntry>} and shoots
     * the note when ready. Does not check that the robot is loaded.
     *
     * @param entrySupplier a lambda that returns the most up-to-date {@code RangeEntry}
     * to prime to before shooting
     * @param readyToShoot lambda that returns whether it's time to shoot. Note that
     * this command will never shoot before the shooter and elevator are ready; this
     * argument allows an extra condition (for example, a button) to be added.
     *
     * @apiNote This command doesn't have a simple end condition; see the code for
     * details.
     */
    public ShootCommand(Supplier<RangeEntry> entrySupplier, BooleanSupplier readyToShoot){
        this(entrySupplier, readyToShoot, () -> 0);
    }

    /**
     * Command that dymanically primes to a {@code Supplier<RangeEntry>} and shoots
     * the note when ready. Does not check that the robot is loaded.
     *
     * @param entrySupplier a lambda that returns the most up-to-date {@code RangeEntry}
     * to prime to before shooting
     *
     * @apiNote This command doesn't have a simple end condition; see the code for
     * details.
     */
    public ShootCommand(Supplier<RangeEntry> entrySupplier){
        this(entrySupplier, () -> true, () -> 0);
    }

    /**
     * Command that static-primes to a {@code RangeEntry} and shoots the note when ready.
     * Does not check that the robot is loaded.
     *
     * @param entry the {@code RangeEntry} to prime to before shooting
     * @param readyToShoot lambda that returns whether it's time to shoot. Note that
     * this command will never shoot before the shooter and elevator are ready; this
     * argument allows an extra condition (for example, a button) to be added.
     *
     * @apiNote This command doesn't have a simple end condition; see the code for
     * details.
     */
    public ShootCommand(RangeEntry entry, BooleanSupplier readyToShoot){
        this(() -> entry, readyToShoot, () -> 0);
    }

    /**
     * Command that static-primes to a {@code RangeEntry} and shoots the note when ready.
     * Does not check that the robot is loaded.
     *
     * @param entry the {@code RangeEntry} to prime to before shooting
     *
     * @apiNote This command doesn't have a simple end condition; see the code for
     * details.
     */
    public ShootCommand(RangeEntry entry){
        this(() -> entry, () -> true, () -> 0);
    }
}
