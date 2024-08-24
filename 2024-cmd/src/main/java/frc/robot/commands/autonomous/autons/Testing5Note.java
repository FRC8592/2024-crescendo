package frc.robot.commands.autonomous.autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.Constants.*;
import frc.robot.commands.autonomous.AutoCommand;

public class Testing5Note extends AutoCommand{
    /**
     * 5-mid-note auto for testing in simulation. This won't fit on the quarter-field and doesn't run in 15s
     */
    public Testing5Note(){
        super(
            new SequentialCommandGroup(
                swerve.commands.followPathCommand(getChoreoTrajectory("Testing5Note_1"), Suppliers.robotRunningOnRed),
                new WaitCommand(1),
                swerve.commands.followPathCommand(getChoreoTrajectory("Testing5Note_2"), Suppliers.robotRunningOnRed),
                new WaitCommand(1),
                swerve.commands.followPathCommand(getChoreoTrajectory("Testing5Note_3"), Suppliers.robotRunningOnRed),
                new WaitCommand(1),
                swerve.commands.followPathCommand(getChoreoTrajectory("Testing5Note_4"), Suppliers.robotRunningOnRed),
                new WaitCommand(1),
                swerve.commands.followPathCommand(getChoreoTrajectory("Testing5Note_5"), Suppliers.robotRunningOnRed)
            ).deadlineWith(leds.commands.blinkCommand(LEDS.RED, 3))
            .andThen(leds.commands.blinkCommand(LEDS.GREEN, 4).withTimeout(0.5))
        );
        setStartStateFromChoreoTrajectory("Testing5Note_1");
    }
}