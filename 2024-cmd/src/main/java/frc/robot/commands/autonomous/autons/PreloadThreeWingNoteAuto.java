package frc.robot.commands.autonomous.autons;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.proxies.*;
import frc.robot.helpers.RangeTable;

public class PreloadThreeWingNoteAuto extends AutoCommand {
    public PreloadThreeWingNoteAuto(){
        super(
            // Preloaded note
            new TimingSimulatedCommand(new PrimeCommand(RangeTable.getSubwoofer(), () -> 0), 1.5),
            new TimingSimulatedCommand(new ShootCommand(), SHOOTER.SHOOT_SCORE_TIME),

            new FollowPathAndScoreCommand(getChoreoTrajectory("PreloadThreeWingNoteAuto_1"), 3, 1.8, false),
            new FollowPathAndScoreCommand(getChoreoTrajectory("PreloadThreeWingNoteAuto_2"), 3, 2.4, false),
            new FollowPathAndScoreCommand(getChoreoTrajectory("PreloadThreeWingNoteAuto_3"), 3, 2.6, false)
        );
        setStartStateFromChoreoPathName("PreloadThreeWingNoteAuto_1");
    }
}
