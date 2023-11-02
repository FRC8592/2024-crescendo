// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.wpilog.*;
import org.littletonrobotics.junction.networktables.*;

public class Robot extends LoggedRobot {
    private DriveTrain drive;
    private XboxController gamePad;
    private Launcher launcher;
    private boolean isTurning;

    @Override
    public void robotInit() {
        Logger.getInstance().recordMetadata("ProjectName", "MyProject");
        if (isReal()) {
            Logger.getInstance().addDataReceiver(new NT4Publisher());
        } else {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.getInstance().setReplaySource(new WPILOGReader(logPath));
            Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }
        Logger.getInstance().start();
        drive = new DriveTrain();
        gamePad = new XboxController(Constants.DRIVER_GAMEPAD_PORT);
        launcher = new Launcher();
        isTurning = false;
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        launcher.setIdle();
        launcher.reZero();
    }

    @Override
    public void teleopPeriodic() {
        launcher.update();
        if (gamePad.getRightBumperPressed() && gamePad.getLeftBumper()) {
            launcher.launch();
        }
        if (gamePad.getAButton()) {
            launcher.reZero();
        }
        if (gamePad.getBButtonReleased()) {
            launcher.setDown();
        }
        if (gamePad.getYButton()) {
            drive.xyDrive(gamePad.getLeftX(), gamePad.getLeftY());
            launcher.up();
        }
        if (gamePad.getYButtonReleased()) {
            launcher.setDown();
            drive.haltDriveTrain();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
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
