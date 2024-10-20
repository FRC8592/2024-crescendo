// package frc.robot.commands;

// import java.util.function.BooleanSupplier;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.*;
// import frc.robot.commands.proxies.NewtonCommand;

// public class LogCommand extends Command {
//     private NewtonCommand commandRunningThis;
//     private BooleanSupplier[] booleansToLog;
//     private DoubleSupplier[] doublesToLog;
//     private String key;

//     public LogCommand(NewtonCommand command, String key, BooleanSupplier... toLog){
//         this.commandRunningThis = command;
//         this.toLog = toLog;
//         this.key = key;
//     }

//     public void initialize(){}
//     public void run(){
//         for()
//         Logger.recordOutput(SHARED.LOG_FOLDER+"/Commands/"+commandRunningThis+"/"+key, toLog);
//     }
// }
