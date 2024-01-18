// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.NewtonSwerve.DriveController;
import com.NewtonSwerve.Gyro.NewtonPigeon;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.autonomous.BaseAuto;
import frc.robot.commands.SparkFlexControl;
import pabeles.concurrency.IntOperatorTask.Min;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.BunnyDropper.States;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private double currentMotorPower = 0.0;
    private double currentMotorPowerTop = 0.0;
    private double currentMotorPowerBottom = 0.0;
    private boolean wasDPADpressed = false;

    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // private XboxController operatorController;
    private XboxController driverController;

    // private Intake intake;
    private NewtonPigeon pigeon;
    //private Swerve swerve;
    // private Shooter shooter;
    // private BunnyDropper dropper;
    // private CrescendoIntake cIntake;

    // private BaseAuto currentAuto;
    // private AutonomousSelector autoSelect;
    // private Timer aimTimer;

    // private boolean wasZeroed = false;
    // private boolean isAiming = false;
    public static Field2d FIELD = new Field2d();
    private static SparkFlexControl leftCrescendoShooter;
    private static SparkFlexControl rightCrescendoShooter;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Logger.getInstance().recordMetadata("ProjectName", "MyProject");
        if (isReal()) {
            Logger.getInstance().addDataReceiver(new WPILOGWriter("/U"));
            Logger.getInstance().addDataReceiver(new NT4Publisher());
        } else {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.getInstance().setReplaySource(new WPILOGReader(logPath));
            Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }
        Logger.getInstance().start();
        SmartDashboard.putData("Auto choices", m_chooser);

        // pigeon = new NewtonPigeon(new Pigeon2(Constants.SWERVE_PIGEON_CAN_ID));
        // swerve = new Swerve(pigeon);
        // shooter = new Shooter();
        // intake = new Intake();
        // dropper = new BunnyDropper();
        driverController = new XboxController(Constants.CONTROLLER_DRIVER_PORT);
        // operatorController = new XboxController(Constants.CONTROLLER_OPERATOR_PORT);
        // aimTimer = new Timer();
        // autoSelect = new AutonomousSelector();
        // cIntake = new CrescendoIntake();
        leftCrescendoShooter = new SparkFlexControl(31);
        rightCrescendoShooter = new SparkFlexControl(29);
        //leftCrescendoShooter.setPID(0.001, 0, 0);
        //rightCrescendoShooter.setPID(0.001, 0, 0);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // if (!isReal()) {
        //     SmartDashboard.putData(FIELD);
        // }
      //  Logger.getInstance().recordOutput("Robot Pose", swerve.getCurrentPos());
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different
     * autonomous modes using the dashboard. The sendable chooser code works with
     * the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
     * chooser code and
     * uncomment the getString line to get the auto name from the text box below the
     * Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure
     * below with additional strings. If using the SendableChooser make sure to add
     * them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // swerve.setSteerAnglesToAbsEncoder();
        // swerve.setTeleopCurrentLimit();

        // if (!wasZeroed) {
        //     pigeon.zeroYaw();
        // }
        SmartDashboard.putNumber("shootSpeed", 0.0);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // //swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        //         new ChassisSpeeds(
        //                 -driverController.getLeftY() * swerve.getMaxTranslateVelo() * 0.15,
        //                 -driverController.getLeftX() * swerve.getMaxTranslateVelo() * 0.15,
        //                 driverController.getRightX() * swerve.getMaxAngularVelo() * 0.15),
        //         swerve.getGyroscopeRotation()));
        /*if (driverController.getAButton()) {
            cIntake.spinPercentOutput(0.2); // Set for the first test
        }
        else if (driverController.getBButton()) {
            cIntake.intakeNote(SmartDashboard.getNumber("Lower Intake Speed", 0),
                    SmartDashboard.getNumber("Upper Intake Speed", 0));
        }
        else {
            cIntake.spinPercentOutput(0);
        }*/
        if (driverController.getRightTriggerAxis() > 0.1){
            double shootSpeed = SmartDashboard.getNumber("shootSpeed", 0.0);
            leftCrescendoShooter.setPercentOutput(-shootSpeed);
            rightCrescendoShooter.setPercentOutput(shootSpeed);;
        }
        else{
            leftCrescendoShooter.stop();
            rightCrescendoShooter.stop();
        }
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
