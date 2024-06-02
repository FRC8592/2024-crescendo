package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.LEDS;
import frc.robot.commands.*;
import frc.robot.commands.proxies.*;
import frc.robot.helpers.*;
import frc.robot.subsystems.*;

public class SystemsCheckAuto extends AutoCommand {
    public SystemsCheckAuto(Swerve swerve, Intake intake, Elevator elevator, Shooter shooter, LEDs leds){
        super(
            swerve.rawDriveCommand(() -> 0.2, () -> 0, () -> 0).withTimeout(0.75),
            swerve.rawDriveCommand(() -> -0.2, () -> 0, () -> 0).withTimeout(0.75),
            swerve.rawDriveCommand(() -> 0, () -> 0.2, () -> 0).withTimeout(0.75),
            swerve.rawDriveCommand(() -> 0, () -> -0.2, () -> 0).withTimeout(0.75),
            swerve.stopCommand(),
            new IntakeCommand(shooter, elevator, intake, leds),
            new WaitCommand(2), // Give humans time to get in position to catch the note
            new PrimeCommand(RangeTable.getSubwoofer(), shooter, elevator, intake, leds, () -> 0).until(() -> shooter.readyToShoot()),
            new ShootCommand(shooter, elevator, intake, leds),
            new WaitCommand(3), // Let whoever caught the note get the note and themselves out of the way
            elevator.setMalleablePositionCommand(Elevator.Positions.AMP).alongWith(
                new WaitForConditionCommand(
                    () -> elevator.isAtTargetPosition(),
                    new WaitCommand(0.5).andThen(
                        elevator.incrementElevatorPositionCommand(0, -ELEVATOR.MANUAL_EXTENSION_SPEED)
                        .until(() -> elevator.getTargetExtension() == ELEVATOR.EXTENSION_METERS_STOWED && elevator.isAtTargetPosition())
                    )
                )
            ),
            new StowCommand(shooter, elevator, intake),
            leds.blinkCommand(LEDS.YELLOW, 3).withTimeout(1),
            leds.partyCommand().withTimeout(0.5),
            leds.blinkCommand(LEDS.GREEN, 4).withTimeout(1)
        );
    }
}
