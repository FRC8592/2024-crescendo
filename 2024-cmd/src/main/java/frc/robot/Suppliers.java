package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.*;
import frc.robot.helpers.RangeTable.RangeEntry;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.helpers.*;

/**
 * Class for storing useful suppliers/lambdas
 */
public final class Suppliers {
    private static Swerve swerve = Swerve.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Elevator elevator = Elevator.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static LEDs leds = LEDs.getInstance();
    private static PoseVision poseVision = PoseVision.getInstance();

    /**
     * {@code getAsBoolean()} returns {@code true} when the robot it running on the red side and
     * {@code false} when on the blue side. Defaults to {@code false} if the alliance color is
     * inaccessible.
     */
    public static final BooleanSupplier robotRunningOnRed = (
        () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
    );

    /**
     * {@code get()} returns the most up-to-date range table entry that will get a shot into the
     * speaker.
     */
    public static final Supplier<RangeEntry> bestRangeEntry = (
        () -> RangeTable.get(poseVision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS))
    );

    /**
     * {@code getAsDouble()} returns the current left-right offset from the speaker
     */
    public static final DoubleSupplier offsetFromSpeakerTag = (
        () -> poseVision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS)
    );

    /**
     * {@code getAsDouble()} returns the output of PoseVision's PID loop to find the
     * speaker tag. If there's no visible tag, returns a negative value (for searching)
     */
    public static final DoubleSupplier aimToSpeakerPidLoopNegativeSearch = (
        () -> poseVision.visual_servo(0, 3, APRILTAG_VISION.SPEAKER_AIM_TAGS, -1.5)
    );

    /**
     * {@code getAsDouble()} returns the output of PoseVision's PID loop to find the
     * speaker tag. If there's no visible tag, returns a positive value (for searching)
     */
    public static final DoubleSupplier aimToSpeakerPidLoopPositiveSearch = (
        () -> poseVision.visual_servo(0, 3, APRILTAG_VISION.SPEAKER_AIM_TAGS, 1.5)
    );

    /**
     * {@code getAsBoolean} returns whether the robot has a note (as determined by
     * the middle beam-break)
     */
    public static final BooleanSupplier robotHasNote = (
        () -> shooter.isMiddleBeamBreakTripped()
    );

    /**
     * {@code getAsBoolean} returns whether the robot is within tolerance of being
     * left-right locked to the center speaker tag
     */
    public static final BooleanSupplier leftRightSpeakerLocked = (
        () -> offsetFromSpeakerTag.getAsDouble() < APRILTAG_VISION.X_ROT_LOCK_TOLERANCE
    );

    /**
     * {@code get()} returns the direction the gyroscope should believe it's facing
     * when the front of the robot is aimed away from the driver station.
     */
    public static final Supplier<Rotation2d> currentRotationOffset = (
        () -> robotRunningOnRed.getAsBoolean() ? SWERVE.RED_PERSPECTIVE_ROTATION : SWERVE.BLUE_PERSPECTIVE_ROTATION
    );

    /**
     * {@code get()} returns the rotational offset from the currently visible speaker
     * tag, or 0 if there is no tag.
     */
    public static final Supplier<Rotation2d> speakerOffset = (
        () -> Rotation2d.fromRadians(
            poseVision.rotationalOffsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS)
        )
    );

    public static void log(){
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Suppliers/RobotHasNote", robotHasNote.getAsBoolean());
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Suppliers/LeftRightSpeakerLocked", leftRightSpeakerLocked.getAsBoolean());
    }
}
