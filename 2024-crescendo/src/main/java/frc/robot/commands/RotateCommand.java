package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.*;

public class RotateCommand extends Command {
    private PIDController turnPID;
    private Swerve drive;
    private Rotation2d target;
    public RotateCommand(Swerve drive, Rotation2d target) {
        turnPID = new PIDController(0.05, 0, 0);
        this.drive = drive;
        this.target = target;
    }
    @Override
    public void initialize() {
        drive.drive(new ChassisSpeeds());
    }
    @Override
    public boolean execute() {
        if(Robot.isReal()){
            drive.drive(new ChassisSpeeds(0, 0,
                    turnPID.calculate(drive.getGyroscopeRotation().getDegrees(), -target.getDegrees())));
            return Math.abs(drive.getGyroscopeRotation().getDegrees() - target.getDegrees()) < 3;
        }
        else{
            Robot.FIELD.setRobotPose(new Pose2d(Robot.FIELD.getRobotPose().getTranslation(), target));
            return true;
        }
    }
    @Override
    public void shutdown() {
        initialize();
    }
}
