package frc.robot.commands.autonomous.autons;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.helpers.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve.DriveModes;

public class SystemsCheckAuto extends AutoCommand {
    /**
     * Run an automated systems-check as an autonomous routine
     */
    public SystemsCheckAuto(){
        super(
            // Drive the swerve in all four directions at low speed to confirm that it works
            swerve.commands.rawDriveCommand(() -> 1, () -> 0, () -> 0, DriveModes.FIELD_RELATIVE).withTimeout(0.75),
            swerve.commands.rawDriveCommand(() -> -1, () -> 0, () -> 0, DriveModes.FIELD_RELATIVE).withTimeout(0.75),
            swerve.commands.rawDriveCommand(() -> 0, () -> 1, () -> 0, DriveModes.FIELD_RELATIVE).withTimeout(0.75),
            swerve.commands.rawDriveCommand(() -> 0, () -> -1, () -> 0, DriveModes.FIELD_RELATIVE).withTimeout(0.75),

            // Rotate in both directions as well
            swerve.commands.rawDriveCommand(() -> 0, () -> 0, () -> 1, DriveModes.FIELD_RELATIVE).withTimeout(0.75),
            swerve.commands.rawDriveCommand(() -> 0, () -> 0, () -> -1, DriveModes.FIELD_RELATIVE).withTimeout(0.75),

            swerve.commands.stopCommand(),

            // Routine to test the intake and shooter
            new IntakeCommand(),
            new WaitCommand(2), // Give humans time to get in position to catch the note
            new ShootCommand(RangeTable.getSubwoofer()),
            new WaitCommand(3), // Let whoever caught the note get the note and themselves out of the way

            //Test the elevator by raising it, lowering the extension as it would in a climb, and stowing
            elevator.commands.setMalleablePositionCommand(Elevator.Positions.AMP).alongWith(
                new WaitUntilCommand(() -> elevator.isAtTargetPosition()).andThen(
                    new WaitCommand(0.5),
                    elevator.commands.incrementElevatorPositionCommand(0, -ELEVATOR.MANUAL_EXTENSION_SPEED)
                    .until(() -> (
                        elevator.getUserDesiredTargetExtension() == ELEVATOR.EXTENSION_METERS_STOWED
                        && elevator.isAtTargetPosition()
                    ))
                )
            ),
            new StowCommand(),

            // Test the LEDs, then blink them green to indicate that the systems check is over
            leds.commands.blinkCommand(LEDS.YELLOW, 3).withTimeout(1),
            leds.commands.partyCommand().withTimeout(0.5),
            leds.commands.blinkCommand(LEDS.GREEN, 4).withTimeout(1)
        );
    }
}
