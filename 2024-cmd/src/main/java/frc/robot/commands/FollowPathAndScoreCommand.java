package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Suppliers;
import frc.robot.commands.proxies.*;
import frc.robot.helpers.RangeTable;
import frc.robot.subsystems.swerve.Swerve.DriveModes;

public class FollowPathAndScoreCommand extends NewtonCommand{
    /**
     * Conveniently combines a path-following command, an intake command, a prime command, and a score command
     * into one package to be used in auto. The robot will not prime or score if the intake fails (failure is
     * defined by reports from the middle beam-break).
     *
     * @param trajectory the path to follow. Will be flipped if the robot is running on the red side
     * @param intakeTimeout the amount of time to wait in the intake before concluding that it has failed. Note
     * that this defined in terms of the <i>start</i> of the command, so make sure to take the timing of the path
     * into account.
     * @param primeDistance the distance to prime for after intaking while following the path
     * @param useVision whether to use vision to line up the shot. Use this if the shot isn't suitably accurate from
     * odometry alone.
     */
    public FollowPathAndScoreCommand(Trajectory trajectory, double intakeTimeout, double primeDistance, boolean useVision){
        super(
            ( // This group of commands runs a path-follow command while intaking, then primes for however
              // much longer the path takes to finish (or doesn't prime at all if the intake takes longer
              // than the path).
                swerve.commands.followPathCommand(trajectory, Suppliers.robotRunningOnRed).alongWith(
                    new IntakeCommand().withTimeout(intakeTimeout).asProxy() // Proxy for the "!intake.currentlyCommanded()" below
                ).deadlineWith(
                    new WaitUntilCommand(() -> !intake.currentlyCommanded()).andThen(
                        new TimingSimulatedCommand(
                            new PrimeCommand(RangeTable.get(primeDistance), () -> 0).onlyIf(Suppliers.robotHasNote), 2
                        )
                    )
                )
            ).andThen(
                ( // This block of commands runs the (optional) extra vision prime-and-aim and shoots
                    new ShootCommand(
                        useVision ? Suppliers.bestRangeEntry : () -> RangeTable.get(primeDistance),
                        Suppliers.leftRightSpeakerLocked
                    ).deadlineWith(
                        // This snapToCommand aims the robot at the speaker
                        swerve.commands.snapToCommand(() -> 0, () -> 0, Suppliers.speakerOffset, DriveModes.FIELD_RELATIVE)
                    )
                ).onlyIf(Suppliers.robotHasNote) // <-- This disables the optional vision prime/aim and the shot if there isn't a note,
                                                 // which saves a significant amount of time
            )
        );
    }
}