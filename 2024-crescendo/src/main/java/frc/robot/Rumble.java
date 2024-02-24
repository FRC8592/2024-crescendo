package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Rumble {
    private XboxController driver;
    private XboxController operator;

    public Rumble(XboxController d, XboxController o) {
        this.driver = d;
        this.operator = o;
    }

    public void collectRangeRumble(int cameraAngle) {
        return;
    }
}
