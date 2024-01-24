package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.NewtonSwerve.Gyro.NewtonPigeon;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.autonomous.*;

public class Robot extends LoggedRobot {

    public static final double JOYSTICK_DEADBAND = 0.01;
    private static final String LOG_PATH = "CustomLogs/Robot/";

    public static Field2d FIELD = new Field2d();

    private XboxController driverController;
    private XboxController operatorController;

    private BaseAuto currentAuto;
    private AutonomousSelector autoSelect;

    private NewtonPigeon pigeon;
    private Swerve swerve;


    @Override
    public void robotInit() {
        Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }

        // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        Logger.start();
        autoSelect = new AutonomousSelector();
        pigeon = new NewtonPigeon(new Pigeon2(Swerve.PIGEON_CAN_ID));
        swerve = new Swerve(pigeon);
        driverController = new XboxController(Hardware.DRIVER_PORT);
        operatorController = new XboxController(Hardware.OPERATOR_PORT);
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() {
        currentAuto = autoSelect.getSelectedAutonomous();
        currentAuto.addModules(swerve, null /* vision */);
        currentAuto.initialize();
        swerve.resetEncoder();
        swerve.resetPose(currentAuto.getStartPose());
        swerve.setSteerAnglesToAbsEncoder();
        swerve.setAutoCurrentLimit();
    }

    @Override
    public void autonomousPeriodic() {
        currentAuto.periodic();
    }

    @Override
    public void teleopInit() {
        swerve.setSteerAnglesToAbsEncoder();
        swerve.setTeleopCurrentLimit();
    }

    @Override
    public void teleopPeriodic() {
        double driveTranslateY = driverController.getLeftY();
        double driveTranslateX = driverController.getLeftX();
        double robotRotationSpeed = driverController.getRightX();
        boolean slowMode = driverController.getRightBumper();
        ChassisSpeeds currentSpeeds;
        if (slowMode) {
            currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                            driveTranslateY * Swerve.TRANSLATE_POWER_SLOW * swerve.getMaxTranslateVelo(),
                            driveTranslateX * Swerve.TRANSLATE_POWER_SLOW * swerve.getMaxTranslateVelo(),
                            robotRotationSpeed * Swerve.ROTATE_POWER_SLOW * swerve.getMaxAngularVelo()),
                    swerve.getGyroscopeRotation());
        }
        else {
            currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                            driveTranslateY * Swerve.TRANSLATE_POWER_FAST * swerve.getMaxTranslateVelo(),
                            driveTranslateX * Swerve.TRANSLATE_POWER_FAST * swerve.getMaxTranslateVelo(),
                            robotRotationSpeed * Swerve.ROTATE_POWER_FAST * swerve.getMaxAngularVelo()),
                    swerve.getGyroscopeRotation());
        }
        swerve.drive(currentSpeeds);
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}