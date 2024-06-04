package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.LEDS;
import frc.robot.commands.*;
import frc.robot.commands.proxies.*;
import frc.robot.helpers.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class SystemsCheckAuto extends AutoCommand {
    public SystemsCheckAuto(Swerve swerve, Intake intake, Elevator elevator, Shooter shooter, LEDs leds){
        super(
            swerve.commands.rawDriveCommand(() -> 0.2, () -> 0, () -> 0).withTimeout(0.75),
            swerve.commands.rawDriveCommand(() -> -0.2, () -> 0, () -> 0).withTimeout(0.75),
            swerve.commands.rawDriveCommand(() -> 0, () -> 0.2, () -> 0).withTimeout(0.75),
            swerve.commands.rawDriveCommand(() -> 0, () -> -0.2, () -> 0).withTimeout(0.75),
            swerve.commands.stopCommand(),
            new IntakeCommand(shooter, elevator, intake, leds),
            new WaitCommand(2), // Give humans time to get in position to catch the note
            new PrimeCommand(RangeTable.getSubwoofer(), shooter, elevator, intake, leds, () -> 0).until(() -> shooter.readyToShoot()),
            new ShootCommand(shooter, elevator, intake, leds),
            new WaitCommand(3), // Let whoever caught the note get the note and themselves out of the way
            elevator.commands.setMalleablePositionCommand(Elevator.Positions.AMP).alongWith(
                new WaitForConditionCommand(
                    () -> elevator.isAtTargetPosition(),
                    new WaitCommand(0.5).andThen(
                        elevator.commands.incrementElevatorPositionCommand(0, -ELEVATOR.MANUAL_EXTENSION_SPEED)
                        .until(
                            () -> elevator.getTargetExtension() == ELEVATOR.EXTENSION_METERS_STOWED
                            && elevator.isAtTargetPosition()
                        )
                    )
                )
            ),
            new StowCommand(shooter, elevator, intake),
            leds.commands.blinkCommand(LEDS.YELLOW, 3).withTimeout(1),
            leds.commands.partyCommand().withTimeout(0.5),
            leds.commands.blinkCommand(LEDS.GREEN, 4).withTimeout(1)
        );
    }
}
