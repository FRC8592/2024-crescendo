package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DELETABLE_BurrowLineup {
    Vision vision;
    Swerve swerve;
    public DELETABLE_BurrowLineup(Vision vision, Swerve swerve){
        this.vision = vision;
        this.swerve = swerve;
    }

    public void BurrowLineupUpdate(){
        swerve.drive(new ChassisSpeeds(vision.getOffsetAngleDegrees(),vision.distanceToTarget(),0));
    }
}
