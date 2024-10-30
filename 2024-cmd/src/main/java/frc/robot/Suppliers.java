package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
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
import edu.wpi.first.util.WPISerializable;

/**
 * Class for storing useful suppliers/lambdas
 */
public final class Suppliers {
    public static class LoggedWPILibSupplier<T extends WPISerializable> implements Supplier<T>{
        private Supplier<T> supplier;
        private String name;
        public LoggedWPILibSupplier(Supplier<T> supplier, String name){this.supplier = supplier; this.name = name;}
        public T get(){T t = supplier.get(); Logger.recordOutput(SUPPLIERS.LOG_PATH+name, t); return t;}
    }
    public static class LoggedEnumSupplier<E extends Enum<E>> implements Supplier<E>{
        private Supplier<E> supplier;
        private String name;
        public LoggedEnumSupplier(Supplier<E> supplier, String name){this.supplier = supplier; this.name = name;}
        public E get(){E e = supplier.get(); Logger.recordOutput(SUPPLIERS.LOG_PATH+name, e); return e;}
    }
    public static class LoggedBooleanSupplier implements BooleanSupplier{
        private BooleanSupplier supplier;
        private String name;
        public LoggedBooleanSupplier(BooleanSupplier supplier, String name){this.supplier = supplier; this.name = name;}
        public boolean getAsBoolean(){boolean b = supplier.getAsBoolean(); Logger.recordOutput(SUPPLIERS.LOG_PATH+name, b); return b;}
    }
    public static class LoggedDoubleSupplier implements DoubleSupplier{
        private DoubleSupplier supplier;
        private String name;
        public LoggedDoubleSupplier(DoubleSupplier supplier, String name){this.supplier = supplier; this.name = name;}
        public double getAsDouble(){double d = supplier.getAsDouble(); Logger.recordOutput(SUPPLIERS.LOG_PATH+name, d); return d;}
    }
    public static class LoggedIntSupplier implements IntSupplier{
        private IntSupplier supplier;
        private String name;
        public LoggedIntSupplier(IntSupplier supplier, String name){this.supplier = supplier; this.name = name;}
        public int getAsInt(){int d = supplier.getAsInt(); Logger.recordOutput(name, d); return d;}
    }

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
    public static final LoggedBooleanSupplier robotRunningOnRed = new LoggedBooleanSupplier(
        () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red,
        "RobotRunningOnRed"
    );

    /**
     * {@code get()} returns the most up-to-date range table entry that will get a shot into the
     * speaker.
     */
    // NOTE: We don't use any special logger supplier here because all the data we need are logged
    // elsewhere and it'd be too much of a pain to make it work.
    public static final Supplier<RangeEntry> bestRangeEntry = (
        () -> RangeTable.get(poseVision.distanceToAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS))
    );

    /**
     * {@code getAsDouble()} returns the current left-right offset from the speaker
     */
    public static final DoubleSupplier offsetFromSpeakerTag = new LoggedDoubleSupplier(
        () -> poseVision.offsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS),
        "OffsetFromSpeakerTag"
    );

    /**
     * {@code getAsDouble()} returns the output of PoseVision's PID loop to find the
     * speaker tag. If there's no visible tag, returns a negative value (for searching)
     */
    public static final DoubleSupplier aimToSpeakerPidLoopNegativeSearch = new LoggedDoubleSupplier(
        () -> poseVision.visual_servo(0, 3, APRILTAG_VISION.SPEAKER_AIM_TAGS, -1.5),
        "AimToSpeakerPIDLoopNegativeSearch"
    );

    /**
     * {@code getAsDouble()} returns the output of PoseVision's PID loop to find the
     * speaker tag. If there's no visible tag, returns a positive value (for searching)
     */
    public static final DoubleSupplier aimToSpeakerPidLoopPositiveSearch = new LoggedDoubleSupplier(
        () -> poseVision.visual_servo(0, 3, APRILTAG_VISION.SPEAKER_AIM_TAGS, 1.5),
        "AimToSpeakerPIDLoopPositiveSearch"
    );

    /**
     * {@code getAsBoolean} returns whether the robot has a note (as determined by
     * the middle beam-break)
     */
    public static final BooleanSupplier robotHasNote = new LoggedBooleanSupplier(
        () -> shooter.isMiddleBeamBreakTripped(),
        "RobotHasNote"
    );

    /**
     * {@code getAsBoolean} returns whether the robot is within tolerance of being
     * left-right locked to the center speaker tag
     */
    public static final BooleanSupplier leftRightSpeakerLocked = new LoggedBooleanSupplier(
        () -> offsetFromSpeakerTag.getAsDouble() < APRILTAG_VISION.X_ROT_LOCK_TOLERANCE,
        "LeftRightSpeakerLocked"
    );

    /**
     * {@code get()} returns the direction the gyroscope should believe it's facing
     * when the front of the robot is aimed away from the driver station.
     */
    public static final Supplier<Rotation2d> currentGyroscopeRotationOffset = new LoggedWPILibSupplier<Rotation2d>(
        () -> robotRunningOnRed.getAsBoolean() ? SWERVE.RED_PERSPECTIVE_ROTATION : SWERVE.BLUE_PERSPECTIVE_ROTATION,
        "CurrentGyroscopeRotationOffset"
    );

    /**
     * {@code get()} returns the rotational offset from the currently visible speaker
     * tag, or 0 if there is no tag.
     */
    public static final Supplier<Rotation2d> rotationalSpeakerOffset = new LoggedWPILibSupplier<Rotation2d>(
        () -> Rotation2d.fromRadians(poseVision.rotationalOffsetFromAprilTag(APRILTAG_VISION.SPEAKER_AIM_TAGS)),
        "RotationalSpeakerOffset"
    );
}
