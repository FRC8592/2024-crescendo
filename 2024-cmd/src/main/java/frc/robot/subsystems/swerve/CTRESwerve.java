package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import frc.robot.Suppliers;
import frc.robot.Constants.SWERVE;

public class CTRESwerve extends SwerveDrivetrain{
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.FieldCentric fieldRelativeDrive;
    private final SwerveRequest.RobotCentric robotRelativeDrive;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    /**
     * Creates a new CTRESwerve (a class dedicated to tucking away the complexity of Phoenix 6)
     * @param driveTrainConstants the constants for the drivetrain as a whole
     * @param commonModuleConstants the shared constants for all modules
     * @param moduleConstants the module constants, generated from commonModuleConstants
     */
    public CTRESwerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstantsFactory commonModuleConstants, SwerveModuleConstants... moduleConstants) {
        super(driveTrainConstants, moduleConstants);

        // These two requests can be combined with a ChassisSpeeds (see getRequest() below) to drive the swerve
        fieldRelativeDrive = (
            new SwerveRequest.FieldCentric()
            .withDeadband(commonModuleConstants.SpeedAt12VoltsMps * 0.001)
            .withRotationalDeadband(SWERVE.MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND * 0.001)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        );
        robotRelativeDrive = (
            new SwerveRequest.RobotCentric()
            .withDeadband(commonModuleConstants.SpeedAt12VoltsMps * 0.001)
            .withRotationalDeadband(SWERVE.MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND * 0.001)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        );

        if (!Robot.isReal()) {
            startSimThread();
        }
    }

    /**
     * Request a {@code ChassisSpeeds} of the swerve with control over whether
     * the robot is field- or robot-relative
     *
     * @param speeds the {@code ChassisSpeeds} object to send to the drivetrain
     * @param isFieldRelative whether to drive the robot field-relative. If this
     * is {@code false}, the drivetrain will run robot-relative instead
     */
    public void drive(ChassisSpeeds speeds, boolean isFieldRelative) {
        this.setControl(
            isFieldRelative ? getRequest(speeds, fieldRelativeDrive) : getRequest(speeds, robotRelativeDrive)
        );
    }

    /**
     * Request a {@code ChassisSpeeds} of the drivetrain, field-relative
     *
     * @param speeds the {@code ChassisSpeeds} object to request
     */
    public void drive(ChassisSpeeds speeds){
        drive(speeds, true);
    }

    /**
     * Form the swerve's wheels in an X pattern to completely lock it from
     * movement. This should be used sparingly.
     */
    public void brake(){
        this.setControl(brake);
    }

    /**
     * @return a {@code ChassisSpeeds} object representing what the robot is
     * currently doing (this is obtained by measuring the encoder readings
     * from the motors, then using kinematics to calculate what they mean)
     */
    public ChassisSpeeds getCurrentSpeeds(){
        return m_cachedState.speeds;
    }

    /**
     * Define whatever direction the robot is facing as forward
     */
    public void resetHeading(){
        this.setGyroscopeYaw(Suppliers.currentRotationOffset.get());
        this.seedFieldRelative();
    }

    /**
     * @return the current yaw as a {@code Rotation2d} (as measured
     * by the gyroscope)
     */
    public Rotation2d getYaw(){
        return Rotation2d.fromDegrees(
            this.getPigeon2().getYaw().getValueAsDouble()
        );
    }

    /**
     * @return the current robot position, as determined by the odometry
     */
    public Pose2d getCurrentOdometryPosition(){
        return m_cachedState.Pose;
    }

    /**
     * Set the known gyroscope heading (for field-relative)
     * @param yaw a {@code Rotation2d} containing the yaw to
     * set the gyroscope to
     */
    public void setGyroscopeYaw(Rotation2d yaw){
        this.getPigeon2().setYaw(yaw.getDegrees());
    }

    /**
     * Set the odometry pose
     * @param pose a {@code Pose2d} containing the desired known pose
     */
    public void setKnownOdometryPose(Pose2d pose){
        this.seedFieldRelative(pose);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(SWERVE.SIMULATION_LOOP_PERIOD);
    }

    public void periodic() {
        // If we haven't flipped based on our alliance color or if the robot
        // restarts in the middle of a match, check to make sure we've applied
        // the alliance perspective.
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(Suppliers.currentRotationOffset.get());
                hasAppliedOperatorPerspective = true;
            });
        }
    }

    /**
     * Apply a {@code ChassisSpeeds}'s data to a field-relative SwerveRequest
     *
     * @param speeds the {@code ChassisSpeeds} to apply
     * @param baseRequest the {@code SwerveRequest.FieldCentric} add the {@code ChassisSpeeds}'s data to
     *
     * @return the modified SwerveRequest, ready to be send to the drivetrain
     */
    private SwerveRequest getRequest(ChassisSpeeds speeds, SwerveRequest.FieldCentric baseRequest){
        return (
            baseRequest.withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }

    /**
     * Apply a {@code ChassisSpeeds}'s data to a robot-relative SwerveRequest
     *
     * @param speeds the {@code ChassisSpeeds} to apply
     * @param baseRequest the {@code SwerveRequest.RobotCentric} add the {@code ChassisSpeeds}'s data to
     *
     * @return the modified SwerveRequest, ready to be send to the drivetrain
     */
    private SwerveRequest getRequest(ChassisSpeeds speeds, SwerveRequest.RobotCentric baseRequest){
        return (
            baseRequest.withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }

    /**
     * Quick interface for a three-input lambda
     * with all inputs parameterized
     */
    public interface TriConsumer<T1, T2, T3> {
        public void accept(T1 t1, T2 t2, T3 t3);
    }

    /**
     * Modified {@code registerTelemetry()} function that gives more data to the lambda than the default
     * one in CTRE's swerve library
     *
     * @param telemetryFunction a lambda that passes in a {@code SwerveDriveState} current state,
     * {@code SwerveDriveKinematics} kinematics object configured for this swerve, and
     * {@code SwerveModule[]} array of this swerve's modules
     */
    public void registerTelemetry(TriConsumer<SwerveDriveState, SwerveDriveKinematics, SwerveModule[]> telemetryFunction){
        registerTelemetry((swerveDriveState) -> telemetryFunction.accept(swerveDriveState, this.m_kinematics, this.Modules));
    }
}
