package com.NewtonSwerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.NewtonSwerve.Gyro.Gyro;

public class NewtonSwerve {

    public static final double METERS_SECOND_TO_TICKS = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));

    // PASSED IN BY USER
    public final Gyro gyro;
    private final ModuleConfig config;

    /**
     * Swerve module controllers, intialized in the constructor
     */
    private final NewtonModule m_frontLeftModule;
    private final NewtonModule m_frontRightModule;
    private final NewtonModule m_backLeftModule;
    private final NewtonModule m_backRightModule;

    // Odometry object for swerve drive
    protected SwerveDriveOdometry odometry;

    // FALCON SPECIFIC
    public static final double METERS_PER_SECOND_TO_TICKS = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));

    private double MAX_SWERVE_DRIVE_TELEOP_CURRENT = 20.0; // Lower values will reduce acceleration
    private double MAX_SWERVE_DRIVE_AUTO_CURRENT = 30.0; // Lower values will reduce acceleration

    // The maximum angular velocity of the robot in radians per second.
    private double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // Set up the kinematics module based on physical drivetrain characteristics
    private SwerveDriveKinematics m_kinematics;

    public NewtonSwerve(ModuleConfig config, Gyro gyro, SwerveModule frontLeft, SwerveModule frontRight,
            SwerveModule backLeft, SwerveModule backRight) {

        this.gyro = gyro;
        this.config = config;

        double MAX_VELOCITY_METERS_PER_SECOND = config.getMaxVelocityMetersPerSecond();

        // set current limits
        this.MAX_SWERVE_DRIVE_AUTO_CURRENT = !Double.isNaN(config.getAutoCurrentLimit()) ? config.getAutoCurrentLimit()
                : this.MAX_SWERVE_DRIVE_AUTO_CURRENT;
        this.MAX_SWERVE_DRIVE_TELEOP_CURRENT = !Double.isNaN(config.getTeleopCurrentLimit())
                ? config.getTeleopCurrentLimit()
                : this.MAX_SWERVE_DRIVE_TELEOP_CURRENT;

        // grab drivetrain dimensions
        double DRIVETRAIN_LENGTH_METERS = config.getDriveTrainLengthMeters();
        double DRIVETRAIN_WIDTH_METERS = config.getDriveTrainWidthMeters();
        double WHEEL_CIRCUMFERENCE = config.getWheelCircumference();

        MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_WIDTH_METERS / 2.0, DRIVETRAIN_LENGTH_METERS / 2.0);

        m_kinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(DRIVETRAIN_WIDTH_METERS / 2.0, DRIVETRAIN_LENGTH_METERS / 2.0),
                // Front right
                new Translation2d(DRIVETRAIN_WIDTH_METERS / 2.0, -DRIVETRAIN_LENGTH_METERS / 2.0),
                // Back left
                new Translation2d(-DRIVETRAIN_WIDTH_METERS / 2.0, DRIVETRAIN_LENGTH_METERS / 2.0),
                // Back right
                new Translation2d(-DRIVETRAIN_WIDTH_METERS / 2.0, -DRIVETRAIN_LENGTH_METERS / 2.0));

        // setup modules
        this.m_frontLeftModule = new NewtonModule(frontLeft, WHEEL_CIRCUMFERENCE);
        this.m_frontRightModule = new NewtonModule(frontRight, WHEEL_CIRCUMFERENCE);
        this.m_backLeftModule = new NewtonModule(backLeft, WHEEL_CIRCUMFERENCE);
        this.m_backRightModule = new NewtonModule(backRight, WHEEL_CIRCUMFERENCE);

        // intialize odometry
        this.odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(),
                new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(),
                        new SwerveModulePosition(), new SwerveModulePosition() });
    }

    // public NewtonSwerve(ModuleConfig config, Gyro gyro, NewtonModule frontLeft,
    // NewtonModule frontRight, NewtonModule backLeft, NewtonModule backRight){

    // this.gyro = gyro;
    // this.config = config;

    // this.MAX_VELOCITY_METERS_PER_SECOND = config.getMaxVelocityMetersPerSecond();
    // this.MAX_VOLTAGE = config.getNominalVoltage();

    // // set current limits
    // this.MAX_SWERVE_DRIVE_AUTO_CURRENT =
    // !Double.isNaN(config.getAutoCurrentLimit()) ? config.getAutoCurrentLimit() :
    // this.MAX_SWERVE_DRIVE_AUTO_CURRENT;
    // this.MAX_SWERVE_DRIVE_TELEOP_CURRENT =
    // !Double.isNaN(config.getTeleopCurrentLimit()) ?
    // config.getTeleopCurrentLimit() : this.MAX_SWERVE_DRIVE_TELEOP_CURRENT;

    // // grab drivetrain dimensions
    // this.DRIVETRAIN_LENGTH_METERS = config.getDriveTrainLengthMeters();
    // this.DRIVETRAIN_WIDTH_METERS = config.getDriveTrainWidthMeters();
    // this.WHEEL_CIRCUMFERENCE = config.getWheelCircumference();

    // // setup modules
    // this.m_frontLeftModule = frontLeft;
    // this.m_frontRightModule = frontRight;
    // this.m_backLeftModule = backLeft;
    // this.m_backRightModule = backRight;

    // // intialize odometry
    // this.odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(), new
    // SwerveModulePosition[] {new SwerveModulePosition(), new
    // SwerveModulePosition(), new SwerveModulePosition(), new
    // SwerveModulePosition()});
    // }

    public double getMaxTranslateVelocity() {
        return config.getMaxVelocityMetersPerSecond();
    }

    public double getMaxAngularVelocity() {
        return MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    // GYRO METHODS

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is
     * currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        gyro.zeroYaw();
    }

    public Rotation2d getGyroscopeRotation() {
        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise
        // makes the angle increase.
        return Rotation2d.fromDegrees(this.getYaw());
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public double getYaw() {
        return gyro.getYaw();
    }

    // POSE METHODS

    public Pose2d getCurrentPos() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(new Rotation2d(0),
                new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(),
                        new SwerveModulePosition(), new SwerveModulePosition() },
                pose);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, config.getMaxVelocityMetersPerSecond());

        double frontLeftVelo = states[0].speedMetersPerSecond;
        double frontRightVelo = states[1].speedMetersPerSecond;
        double backLeftVelo = states[2].speedMetersPerSecond;
        double backRightVelo = states[3].speedMetersPerSecond;

        m_frontLeftModule.setModule(states[0].angle.getRadians(), metersPerSecondToTicks(frontLeftVelo));
        m_frontRightModule.setModule(states[1].angle.getRadians(), metersPerSecondToTicks(frontRightVelo));
        m_backLeftModule.setModule(states[2].angle.getRadians(), metersPerSecondToTicks(backLeftVelo));
        m_backRightModule.setModule(states[3].angle.getRadians(), metersPerSecondToTicks(backRightVelo));

        this.odometry.update(this.getGyroscopeRotation(),
                new SwerveModulePosition[] { m_frontLeftModule.getModulePosition(),
                        m_frontRightModule.getModulePosition(), m_backLeftModule.getModulePosition(),
                        m_backRightModule.getModulePosition() });
    }

    // zero the absolute encoder for all modules
    public void resetEncoder() {
        m_frontLeftModule.resetThrottleEncoder();
        m_frontRightModule.resetThrottleEncoder();
        m_backLeftModule.resetThrottleEncoder();
        m_backRightModule.resetThrottleEncoder();
    }

    public double metersPerSecondToTicks(double input) {
        return input * METERS_SECOND_TO_TICKS;
    }

    public double ticksToMetersPerSecond(double input) {
        return input / METERS_SECOND_TO_TICKS;
    }

    public void resetSteerAngles() {
        m_frontLeftModule.resetAbsoluteAngle();
        m_frontRightModule.resetAbsoluteAngle();
        m_backLeftModule.resetAbsoluteAngle();
        m_backRightModule.resetAbsoluteAngle();
    }

    // SET CURRENT LIMIT
    private void setThrottleCurrentLimit(double currentLimit) {
        m_frontLeftModule.setThrottleCurrentLimit(currentLimit);
        m_frontRightModule.setThrottleCurrentLimit(currentLimit);
        m_backLeftModule.setThrottleCurrentLimit(currentLimit);
        m_backRightModule.setThrottleCurrentLimit(currentLimit);
    }

    public void setTeleopCurrentLimit() {
        setThrottleCurrentLimit(MAX_SWERVE_DRIVE_TELEOP_CURRENT);
    }

    public void setAutoCurrentLimit() {
        setThrottleCurrentLimit(MAX_SWERVE_DRIVE_AUTO_CURRENT);
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public double[] getThrottleAppliedCurrent() {
        double frontLeftCurrent = m_frontLeftModule.getAppliedCurrent();
        double frontRightCurrent = m_frontRightModule.getAppliedCurrent();
        double backLeftCurrent = m_backLeftModule.getAppliedCurrent();
        double backRightCurrent = m_backRightModule.getAppliedCurrent();

        return new double[] { frontLeftCurrent, frontRightCurrent, backLeftCurrent, backRightCurrent };
    }

    public double[] getThrottleAppliedVelocity() {
        double frontLeftVelo = ticksToMetersPerSecond(m_frontLeftModule.getThrottleVelocity(null));
        double frontRightVelo = ticksToMetersPerSecond(m_frontRightModule.getThrottleVelocity(null));
        double backLeftVelo = ticksToMetersPerSecond(m_backLeftModule.getThrottleVelocity(null));
        double backRightVelo = ticksToMetersPerSecond(m_backRightModule.getThrottleVelocity(null));

        return new double[] { Math.abs(frontLeftVelo), Math.abs(frontRightVelo), Math.abs(backLeftVelo),
                Math.abs(backRightVelo) };
    }
}
