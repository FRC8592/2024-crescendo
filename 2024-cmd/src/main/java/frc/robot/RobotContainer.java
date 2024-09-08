// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.proxies.OverrideEverythingCommand;
import frc.robot.subsystems.singlemotor.SingleMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {
    // The robot's subsystems
    private SingleMotor singleMotor;


    // Controllers
    private final CommandXboxController controller = new CommandXboxController(
        CONTROLLERS.CONTROLLER_PORT
    );

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        singleMotor = SingleMotor.instantiate();

        configureDefaults();
        configureBindings();
    }

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults(){
        setDefaultCommand(
            singleMotor,
            singleMotor.commands.stopMotorCommand().withInterruptBehavior(
                InterruptionBehavior.kCancelSelf
            )
        );
    }

    /**
     * Configure all button bindings
     */
    private void configureBindings() {
        controller.a().whileTrue(singleMotor.commands.driveMotorWithVelocityCommand(500));
        controller.b().whileTrue(
            singleMotor.commands.driveMotorWithVelocityCommand(() -> (
                (int)(controller.getLeftY()*1000)
            )
        ));

        controller.x().onTrue(
            singleMotor.commands.sysIdQuasistatic(Direction.kForward)
            .andThen(new WaitCommand(2)).andThen(singleMotor.commands.sysIdQuasistatic(Direction.kReverse))
            .andThen(new WaitCommand(2)).andThen(singleMotor.commands.sysIdDynamic(Direction.kForward))
            .andThen(new WaitCommand(2)).andThen(singleMotor.commands.sysIdDynamic(Direction.kReverse))
        );

        controller.y().whileTrue(
            singleMotor.commands.driveMotorWithVelocityCommand(100)
        );
    }

    /**
     * Set the default command of a subsystem (what to run if no other command requiring it is running).
     * <p> NOTE: all subsystems also have a setDefaultCommand method; this version includes a check for
     * default commands that cancel incoming commands that require the subsystem. Unless you're sure
     * of what you're doing, you should use this one.
     *
     * @param subsystem the subsystem to apply the default command to
     * @param command to command to set as default
     */
    private void setDefaultCommand(SubsystemBase subsystem, Command command){
        if(command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf){
            subsystem.setDefaultCommand(command);
        }
        else{
            //If you want to force-allow setting a cancel-incoming default command, directly call `subsystem.setDefaultCommand()` instead
            throw new UnsupportedOperationException("Can't set a default command that cancels incoming!");
        }
    }
}
