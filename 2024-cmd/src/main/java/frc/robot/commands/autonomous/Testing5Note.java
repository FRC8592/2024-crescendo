package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LEDS;
import frc.robot.subsystems.*;

public class Testing5Note extends AutoCommand{
    public Testing5Note(Swerve swerve, Intake intake, Elevator elevator, Shooter shooter, LEDs leds){
        super(
            new SequentialCommandGroup(
                swerve.followPathCommand(getChoreoTrajectory("Testing5Note_1"), flipPathToRedSide),
                new WaitCommand(1),
                swerve.followPathCommand(getChoreoTrajectory("Testing5Note_2"), flipPathToRedSide),
                new WaitCommand(1),
                swerve.followPathCommand(getChoreoTrajectory("Testing5Note_3"), flipPathToRedSide),
                new WaitCommand(1),
                swerve.followPathCommand(getChoreoTrajectory("Testing5Note_4"), flipPathToRedSide),
                new WaitCommand(1),
                swerve.followPathCommand(getChoreoTrajectory("Testing5Note_5"), flipPathToRedSide)
            ).alongWith(leds.blinkCommand(LEDS.RED, 3))
        );
    }
}