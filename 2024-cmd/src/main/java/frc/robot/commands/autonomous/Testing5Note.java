package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LEDS;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class Testing5Note extends AutoCommand{
    /**
     * 5-mid-note auto for testing in simulation. This won't fit on the quarter-field and doesn't run in 15s
     */
    public Testing5Note(Swerve swerve, Intake intake, Elevator elevator, Shooter shooter, LEDs leds){
        super(
            new SequentialCommandGroup(
                swerve.commands.followPathCommand(getChoreoTrajectory("Testing5Note_1"), flipPathToRedSide),
                new WaitCommand(1),
                swerve.commands.followPathCommand(getChoreoTrajectory("Testing5Note_2"), flipPathToRedSide),
                new WaitCommand(1),
                swerve.commands.followPathCommand(getChoreoTrajectory("Testing5Note_3"), flipPathToRedSide),
                new WaitCommand(1),
                swerve.commands.followPathCommand(getChoreoTrajectory("Testing5Note_4"), flipPathToRedSide),
                new WaitCommand(1),
                swerve.commands.followPathCommand(getChoreoTrajectory("Testing5Note_5"), flipPathToRedSide)
            ).alongWith(leds.commands.blinkCommand(LEDS.RED, 3))
        );
    }
}