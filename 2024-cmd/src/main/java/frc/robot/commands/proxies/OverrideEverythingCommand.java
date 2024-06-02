// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.proxies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class OverrideEverythingCommand extends Command {
    Command command;

    /**
     * Proxy command that cancels all scheduled commands before
     * scheduling the passed-in command.
     *
     * @param command the command to run
     */
    public OverrideEverythingCommand(Command command) {
        this.command = command;
    }

    public void initialize() {
    }

    public void execute() {
        CommandScheduler.getInstance().cancelAll();
    }

    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(command);
    }

    public boolean isFinished() {
        return false;
    }
}
