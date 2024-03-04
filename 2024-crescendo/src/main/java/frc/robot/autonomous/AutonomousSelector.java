package frc.robot.autonomous;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.autonomous.autons.*;

public class AutonomousSelector {
    public ShuffleboardTab autonTab = Shuffleboard.getTab("Auton Configuration");
    private SendableChooser<Class<?>> autonChooser = new SendableChooser<>();
    private GenericEntry delayEntry;

    public Class<?>[] autos = {
        //putting all created autos in auto selector
        
        TwoNoteVisionAuto.class,
        ThreeWingNoteAuto.class,
        SixNoteAuto.class,
        NoteStealAuto.class,
        TwoNoteAuto.class,
            RedThreeWingNoteVisionAuto.class,
            ThreeWingNoteVisionAuto.class,
        OneNoteAuto.class,
        Left2MidnotesAuto.class,
        DONTSELECT_RotateTestAuto.class
        // IWillNameThisLaterAuto.class,
    };

    public AutonomousSelector() {
        autonChooser.setDefaultOption("DEFAULT - DO NOTHING", DoNothingAuto.class);
        for (Class<?> auto : autos) {
            autonChooser.addOption(auto.getSimpleName(), auto);
        }

        autonTab.add("Choose Autonomous", autonChooser)
            .withPosition(3, 2)
            .withSize(4, 2);

        delayEntry = autonTab.add("Autonomous Delay", 0d)
            .withPosition(4, 1)
            .withSize(2, 1)
            .getEntry();
    }

    public double getDelay() {
        return delayEntry.getDouble(0.0);
    }

    public BaseAuto getSelectedAutonomous() {
        try {
            BaseAuto selected = (BaseAuto) autonChooser.getSelected().getDeclaredConstructor().newInstance();
            return selected;
        } catch (Exception e) {
             System.out.println(e.getMessage());
             return null;
        }
        // BaseAuto selected = (BaseAuto) autonChooser.getSelected().getDeclaredConstructor().newInstance();
        // return selected;
    }
}
