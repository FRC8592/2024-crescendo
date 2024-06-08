package frc.robot.commands.autonomous;

import frc.robot.Constants.*;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PrimeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.proxies.TimingSimulatedCommand;
import frc.robot.helpers.RangeTable;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class PreloadThreeWingNoteAuto extends AutoCommand {
    public PreloadThreeWingNoteAuto(Swerve swerve, Intake intake, Elevator elevator, Shooter shooter, LEDs leds){
        super(
            swerve.commands.followPathCommand(
                getChoreoTrajectory("PreloadThreeWingNoteAuto_1"), flipPathToRedSide
            ).alongWith(
                new TimingSimulatedCommand(new IntakeCommand(shooter, elevator, intake, leds))
            ),

            // Wait 2 seconds if simulated (notice the 2 on the end of the line)
            new TimingSimulatedCommand(new PrimeCommand(RangeTable.get(1.4), shooter, elevator, intake, leds, () -> 0), 2),
            new TimingSimulatedCommand(new ShootCommand(shooter, elevator, intake, leds), SHOOTER.SHOOT_SCORE_TIME),

            swerve.commands.followPathCommand(
                getChoreoTrajectory("PreloadThreeWingNoteAuto_2"), flipPathToRedSide
            ).alongWith(
                new TimingSimulatedCommand(new IntakeCommand(shooter, elevator, intake, leds))
            ),

            new TimingSimulatedCommand(new PrimeCommand(RangeTable.get(1.4), shooter, elevator, intake, leds, () -> 0), 2),
            new TimingSimulatedCommand(new ShootCommand(shooter, elevator, intake, leds), SHOOTER.SHOOT_SCORE_TIME),

            swerve.commands.followPathCommand(
                getChoreoTrajectory("PreloadThreeWingNoteAuto_3"), flipPathToRedSide
            ).alongWith(
                new TimingSimulatedCommand(new IntakeCommand(shooter, elevator, intake, leds))
            ),

            new TimingSimulatedCommand(new PrimeCommand(RangeTable.get(1.4), shooter, elevator, intake, leds, () -> 0), 2),
            new TimingSimulatedCommand(new ShootCommand(shooter, elevator, intake, leds), SHOOTER.SHOOT_SCORE_TIME),

            leds.commands.blinkCommand(LEDS.GREEN, 4).withTimeout(1)
        );
    }
}
