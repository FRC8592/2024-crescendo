package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewtonSubsystem extends SubsystemBase {
    public Command autonomousInitCommand(){
        return Commands.none();
    }
}
