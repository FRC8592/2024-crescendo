package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
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
     * @param primePosition the position to prime for while following the path
     * @param useVision whether to use vision to line up the shot. Use this if the shot isn't suitably accurate from
     * odometry alone.
     */
    public FollowPathAndScoreCommand(Trajectory trajectory, double intakeTimeout, double primePosition, boolean useVision){
        super(
            swerve.commands.followPathCommand(trajectory, Suppliers.robotRunningOnRed).alongWith(
                new IntakeCommand().withTimeout(intakeTimeout)
                .andThen(new TimingSimulatedCommand(
                    new PrimeCommand(RangeTable.get(primePosition), () -> 0).onlyIf(Suppliers.robotHasNote), 2
                ))
            )
            .andThen(
                ( // This block of commands runs the (optional) extra vision prime-and-aim and shoots
                    (
                        new ShootCommand(
                            Suppliers.bestRangeEntry, Suppliers.leftRightSpeakerLocked
                        ).alongWith(
                            // This rawDriveCommand aims the robot at the speaker
                            swerve.commands.rawDriveCommand(
                                () -> 0, () -> 0, Suppliers.aimToSpeakerPidLoopNegativeSearch, DriveModes.ROBOT_RELATIVE
                            )
                        )
                    ).onlyIf(() -> useVision)
                ).onlyIf(Suppliers.robotHasNote) // <-- This disables the optional vision prime/aim and the shot if there isn't a note,
                                                 // which saves a significant amount of time
            )
        );
    }
}