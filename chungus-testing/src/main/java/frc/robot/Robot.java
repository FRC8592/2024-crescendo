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

    private XboxController operatorController;
    private XboxController driverController;

    private Intake intake;
    private NewtonPigeon pigeon;
    private Swerve swerve;
    private Shooter shooter;
    private BunnyDropper dropper;
    private CrescendoIntake cIntake;

    private BaseAuto currentAuto;
    private AutonomousSelector autoSelect;
    private Timer aimTimer;

    private boolean wasZeroed = false;
    private boolean isAiming = false;
    public static Field2d FIELD = new Field2d();

    enum TestStates {
        DRIVETRAIN,
        INTAKE,
        SHOOTER,
        PROTOTYPE_1,
        PROTOTYPE_2,
        NONE
    }

    TestStates testState;

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

        pigeon = new NewtonPigeon(new Pigeon2(Constants.SWERVE_PIGEON_CAN_ID));
        swerve = new Swerve(pigeon);
        shooter = new Shooter();
        intake = new Intake();
        dropper = new BunnyDropper();
        driverController = new XboxController(Constants.CONTROLLER_DRIVER_PORT);
        operatorController = new XboxController(Constants.CONTROLLER_OPERATOR_PORT);
        aimTimer = new Timer();
        autoSelect = new AutonomousSelector();
        cIntake = new CrescendoIntake();
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
        if (!isReal()) {
            SmartDashboard.putData(FIELD);
        }
        Logger.getInstance().recordOutput("Robot Pose", swerve.getCurrentPos());
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
        /*
        pigeon.zeroYaw();
        wasZeroed = true;
        currentAuto = autoSelect.getSelectedAutonomous();
        currentAuto.addModules(swerve, intake, null, shooter, dropper);
        currentAuto.initialize();
        swerve.resetEncoder();
        swerve.resetPose(currentAuto.getStartPose());
        swerve.setSteerAnglesToAbsEncoder();
        swerve.setAutoCurrentLimit();
        dropper.zero();*/

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        //currentAuto.periodic();
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        /*swerve.setSteerAnglesToAbsEncoder();
        swerve.setTeleopCurrentLimit();

        if (!wasZeroed) {
            pigeon.zeroYaw();
        }*/
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        if (driverController.getAButton()) {
            cIntake.spinPercentOutput();
        }
        /*double driveTranslateY = -driverController.getLeftY(); // negative b/c auto starts that way
        double driveTranslateX = -driverController.getLeftX(); // negative b/c auto starts that way
        double robotRotationSpeed = driverController.getRightX();
        boolean slowMode = driverController.getRightBumper();
        boolean intaking = operatorController.getLeftTriggerAxis() > 0.1;
        boolean shootMode = operatorController.getRightTriggerAxis() > 0.1;
        boolean dropBunny = operatorController.getBButton();
        boolean zeroGyro = driverController.getBackButton();
        boolean eject = operatorController.getLeftBumper();
        boolean moveBackAim = driverController.getLeftBumper();

        Logger.getInstance().recordOutput("RawInput/DriveTranslateX", driveTranslateX);
        Logger.getInstance().recordOutput("RawInput/DriveTranslateY", driveTranslateY);
        Logger.getInstance().recordOutput("RawInput/RobotRotationSpeed", robotRotationSpeed);
        Logger.getInstance().recordOutput("RawInput/SlowMode", slowMode);
        Logger.getInstance().recordOutput("RawInput/Intaking", intaking);
        Logger.getInstance().recordOutput("RawInput/ShootMode", shootMode);
        Logger.getInstance().recordOutput("RawInput/ZeroGyro", zeroGyro);
        Logger.getInstance().recordOutput("RawInput/Eject", eject);
        Logger.getInstance().recordOutput("RawInput/MoveBackAim", moveBackAim);

        ChassisSpeeds currentSpeeds;
        shooter.log2();
        // GYRO
        if (zeroGyro) {
            pigeon.zeroYaw();
        }

        // DRIVETRAIN
        if(!isAiming){
            if (!slowMode) {
                Logger.getInstance().recordOutput("Drivetrain State", "JOYSTICK/FAST");
                currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                            driveTranslateY * Constants.SWERVE_TRANSLATE_POWER_FAST * swerve.getMaxTranslateVelo(),
                            driveTranslateX * Constants.SWERVE_TRANSLATE_POWER_FAST * swerve.getMaxTranslateVelo(),
                            robotRotationSpeed * Constants.SWERVE_ROTATE_POWER_FAST * swerve.getMaxAngularVelo()
                        ),
                    swerve.getGyroscopeRotation());
            }
            else {
                Logger.getInstance().recordOutput("Drivetrain State", "JOYSTICK/SLOW");
                currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                            driveTranslateY * Constants.SWERVE_TRANSLATE_POWER_SLOW * swerve.getMaxTranslateVelo(),
                            driveTranslateX * Constants.SWERVE_TRANSLATE_POWER_SLOW * swerve.getMaxTranslateVelo(),
                            robotRotationSpeed * Constants.SWERVE_ROTATE_POWER_SLOW * swerve.getMaxAngularVelo()
                        ),
                    swerve.getGyroscopeRotation());
            }
            if(moveBackAim){
                isAiming=true;
            }
        }
        else{
            aimTimer.start();
            if (aimTimer.get() <= Constants.MOVE_BACK_AIM_TIME) {
                currentSpeeds = new ChassisSpeeds (Constants.MOVE_BACK_AIM_SPEED, 0, 0);
                Logger.getInstance().recordOutput("Drivetrain State", "AIM_BACKUP/MOVING");
            }
            else{
                Logger.getInstance().recordOutput("Drivetrain State", "AIM_BACKUP/DONE");
                currentSpeeds = new ChassisSpeeds(0,0,0);
                aimTimer.stop();
                aimTimer.reset();
                isAiming=false;
            }
        }
        Logger.getInstance().recordOutput("Robot Movement Speed Data (X/Y/Omega)", new double[] {currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond, currentSpeeds.omegaRadiansPerSecond});
        swerve.drive(currentSpeeds);

        // DEAD AXLE POP AND INTAKE
        if (eject) {
            Logger.getInstance().recordOutput("DAP/Intake State", "EJECTING");
            shooter.spinFrontMotorPct(-1.0); // spin backwards
            intake.spinIntakePercentOutput(-1.0); // spin backwards to eject
        }
        else if (intaking) {
            Logger.getInstance().recordOutput("DAP/Intake State", "INTAKING");
            intake.extendIntake();
            shooter.spinFrontMotorPct(0.5); // spin front motor slow while intaking
        }
        else if (shootMode){
            Logger.getInstance().recordOutput("DAP/Intake State", "SHOOTING");
            shooter.shoot(intake);
            // intake.spinRoller(0);
        }
        else{
            Logger.getInstance().recordOutput("DAP/Intake State", "NOTHING");
            intake.retractIntake();
            intake.spinRoller(0);
            shooter.spinFrontMotorPct(0.0);
            shooter.stop();
        }
        intake.update(shootMode);

        // BUNNY DROPPER
        if (dropBunny) {
            Logger.getInstance().recordOutput("Bunny Dropper State", "DROPPING");
            dropper.dropBunny(States.DROP);
        }
        else {
            Logger.getInstance().recordOutput("Bunny Dropper State", "HOLDING");
            dropper.dropBunny(States.HOLD);
        }*/
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
        swerve.setSteerAnglesToAbsEncoder();
        swerve.setTeleopCurrentLimit();

        if (!wasZeroed) {
            pigeon.zeroYaw();
        }
        testState = TestStates.NONE;
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        double SPEED_SCALE = 0.3; // TODO: Universal speed multiplier for testing
        switch (testState) {
            case NONE: // TODO: Nothing
            default:
                SmartDashboard.putString("Current Test", "None");
                break;
            case DRIVETRAIN: // TODO: Use left stick to translate and right stick X to rotate
                swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                                -driverController.getLeftY() * swerve.getMaxTranslateVelo() * SPEED_SCALE,
                                -driverController.getLeftX() * swerve.getMaxTranslateVelo() * SPEED_SCALE,
                                driverController.getRightX() * SPEED_SCALE * swerve.getMaxAngularVelo()),
                        swerve.getGyroscopeRotation()));
                SmartDashboard.putString("Current Test", "Drivetrain");
                break;
            case INTAKE: // TODO: Use the left bumper to retract, the right bumper to extend, and the
                         // left stick Y to spin the intake roller
                if (driverController.getLeftBumperReleased()) {
                    intake.retractIntake(); // state machine
                }
                else if (driverController.getRightBumperReleased()) {
                    intake.extendIntake(); // state machine
                }
                intake.update(true); // actuate the pneumatics (in teleop call this periodically)
                SmartDashboard.putString("Current Test", "Intake");
                break;
            case SHOOTER: // TODO: Use the left stick Y to spin the back motor and the left stick X to
                          // spin the front.
                shooter.TEST_spinBackMotor(driverController.getLeftY() * SPEED_SCALE);
                shooter.spinFrontMotorPct(driverController.getRightY() * SPEED_SCALE);
                SmartDashboard.putString("Current Test", "Shooter");
                break;
            case PROTOTYPE_1:
                swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                        -driverController.getLeftY() * swerve.getMaxTranslateVelo() * SPEED_SCALE,
                        -driverController.getLeftX() * swerve.getMaxTranslateVelo() * SPEED_SCALE,
                        driverController.getRightX() * SPEED_SCALE * swerve.getMaxAngularVelo()),
                swerve.getGyroscopeRotation()));
                if (operatorController.getPOV() == 0 && wasDPADpressed == false){
                    //Increase the speed by .05
                    currentMotorPower += 0.05;
                } else if(operatorController.getPOV() == 180 && wasDPADpressed == false){
                    //Decrease the speed by .05
                    currentMotorPower -= 0.05;
                }
                wasDPADpressed = operatorController.getPOV() != -1;
                currentMotorPower = Math.max(Math.min(currentMotorPower, 1), -1);
                if (operatorController.getAButton() == true){
                    shooter.TEST_spinBackMotor(currentMotorPower);
                } else{
                    shooter.TEST_spinBackMotor(0);
                }
                SmartDashboard.putString("Current Test", "Prototype");
                SmartDashboard.putNumber("Current Motor Power", currentMotorPower);
                break;
            case PROTOTYPE_2:
                swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                        -driverController.getLeftY() * swerve.getMaxTranslateVelo() * SPEED_SCALE,
                        -driverController.getLeftX() * swerve.getMaxTranslateVelo() * SPEED_SCALE,
                        driverController.getRightX() * SPEED_SCALE * swerve.getMaxAngularVelo()),
                swerve.getGyroscopeRotation()));

                if (operatorController.getPOV() == 0 && wasDPADpressed == false && operatorController.getXButton() == true){
                    //Increase the speed by .05
                    currentMotorPowerTop += 0.05;
                } else if(operatorController.getPOV() == 180 && wasDPADpressed == false && operatorController.getXButton() == true){
                    //Decrease the speed by .05
                    currentMotorPowerTop -= 0.05;
                }

                if (operatorController.getPOV() == 0 && wasDPADpressed == false && operatorController.getYButton() == true){
                    //Increase the speed by .05
                    currentMotorPowerBottom += 0.05;
                } else if(operatorController.getPOV() == 180 && wasDPADpressed == false && operatorController.getYButton() == true){
                    //Decrease the speed by .05
                    currentMotorPowerBottom -= 0.05;
                }

                wasDPADpressed = operatorController.getPOV() != -1;
                currentMotorPowerTop = Math.max(Math.min(currentMotorPowerTop, 1), -1);
                currentMotorPowerBottom = Math.max(Math.min(currentMotorPowerBottom, 1), -1);
                if (operatorController.getAButton() == true){
                    shooter.TEST_spinBackMotor(currentMotorPowerTop); //Find which motor top is
                } else{
                    shooter.TEST_spinBackMotor(0);
                }

                if (operatorController.getBButton() == true){
                    intake.spinIntakePercentOutput(currentMotorPowerBottom); //Find which motor bottom is
                } else{
                    intake.spinIntakePercentOutput(0);
                }
                SmartDashboard.putString("Current Test", "Prototype");
                SmartDashboard.putNumber("Top Motor Power", currentMotorPowerTop);
                SmartDashboard.putNumber("Bottom Motor Power", currentMotorPowerBottom);
        }

            // case DROPPER: //TODO: Use the left stick Y to spin the bunny dropper
            // dropper.TEST_spinBunnyDropper(driverController.getLeftY() * speed);
            // SmartDashboard.putString("Current Test", "Bunny Dropper");
            // break;
        if (driverController.getAButtonReleased()) {
            switch (testState) {
                case NONE:
                    testState = TestStates.DRIVETRAIN;
                    break;
                case DRIVETRAIN:
                    testState = TestStates.INTAKE;
                    break;
                case INTAKE:
                    testState = TestStates.SHOOTER;
                    break;
                default:
                case SHOOTER:
                    testState = TestStates.PROTOTYPE_1;
                    break;
                case PROTOTYPE_1:
                    currentMotorPower = 0.0;
                    testState = TestStates.PROTOTYPE_2;
                    break;
                case PROTOTYPE_2:
                    currentMotorPowerTop = 0.0;
                    currentMotorPowerBottom = 0.0;
                    testState = TestStates.NONE;
                    break;

                // case DROPPER:
                // testState=TestStates.NONE;
                // break;
            }
        }

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
