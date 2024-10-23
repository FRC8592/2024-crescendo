package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Controls {
    /**
     * Enum for the different sets of controls (different drivers,
     * different DS configurations, etc)
     */
    protected enum ControlSets{
        MAIN_TELEOP,
        DISABLED,
    }

    private static CommandXboxController driverController;
    private static CommandXboxController operatorController;

    protected static Trigger slowMode = new Trigger(() -> false);
    protected static Trigger robotRelative = new Trigger(() -> false);
    protected static Trigger zeroGryoscope = new Trigger(() -> false);

    protected static Trigger runVoltageMode = new Trigger(() -> false);
    protected static Trigger passAim = new Trigger(() -> false);
    protected static Trigger autoAim = new Trigger(() -> false);

    protected static Trigger snapForward = new Trigger(() -> false);
    protected static Trigger snapBack = new Trigger(() -> false);
    protected static Trigger snapLeft = new Trigger(() -> false);
    protected static Trigger snapRight = new Trigger(() -> false);

    protected static Trigger sysIdTests = new Trigger(() -> false);
    protected static Trigger noteRequest = new Trigger(() -> false);

    protected static Trigger visionShoot = new Trigger(() -> false);
    protected static Trigger podiumShoot = new Trigger(() -> false);
    protected static Trigger trapShoot = new Trigger(() -> false);
    protected static Trigger score = new Trigger(() -> false); // The score button is also used for a subwoofer shoot
    protected static Trigger passThrough = new Trigger(() -> false);

    protected static Trigger ampScore = new Trigger(() -> false);

    protected static Trigger stow = new Trigger(() -> false);
    protected static Trigger intake = new Trigger(() -> false);
    protected static Trigger outake = new Trigger(() -> false);
    protected static Trigger climb = new Trigger(() -> false);
    protected static Trigger extendElevator = new Trigger(() -> false);
    protected static Trigger retractElevator = new Trigger(() -> false);

    /**
     * Add the controller objects to the Controls class. This MUST be called before
     * applying a control set.
     *
     * @param driverController
     * @param operatorController
     */
    protected static void addControllers(CommandXboxController driverController, CommandXboxController operatorController){
        Controls.driverController = driverController;
        Controls.operatorController = operatorController;
    }

    /**
     * Change the variables in the Controls class to match the specified
     * control set. Note that this doesn't edit or remove bindings.
     *
     * @param set the control set to apply
     */
    protected static void applyControlSet(ControlSets set){
        switch (set) {
            default: case DISABLED:
                slowMode = new Trigger(() -> false);
                robotRelative = new Trigger(() -> false);
                zeroGryoscope = new Trigger(() -> false);

                runVoltageMode = new Trigger(() -> false);
                passAim = new Trigger(() -> false);
                autoAim = new Trigger(() -> false);

                snapForward = new Trigger(() -> false);
                snapBack = new Trigger(() -> false);
                snapLeft = new Trigger(() -> false);
                snapRight = new Trigger(() -> false);

                sysIdTests = new Trigger(() -> false);
                noteRequest = new Trigger(() -> false);

                visionShoot = new Trigger(() -> false);
                podiumShoot = new Trigger(() -> false);
                trapShoot = new Trigger(() -> false);
                score = new Trigger(() -> false);
                passThrough = new Trigger(() -> false);

                ampScore = new Trigger(() -> false);

                stow = new Trigger(() -> false);
                intake = new Trigger(() -> false);
                outake = new Trigger(() -> false);
                climb = new Trigger(() -> false);
                extendElevator = new Trigger(() -> false);
                retractElevator = new Trigger(() -> false);
                break;

            case MAIN_TELEOP:
                slowMode = driverController.rightBumper();
                robotRelative = driverController.leftBumper();
                zeroGryoscope = driverController.back();

                runVoltageMode = driverController.a();
                passAim =  driverController.y();
                autoAim = driverController.leftTrigger(0.1);

                snapForward = driverController.pov(0);
                snapBack = driverController.pov(180);
                snapLeft = driverController.pov(90);
                snapRight = driverController.pov(270);

                sysIdTests = driverController.start();
                noteRequest = operatorController.back();

                visionShoot = operatorController.rightBumper();
                podiumShoot = operatorController.b();
                trapShoot = operatorController.start();
                score = driverController.rightTrigger(0.1);
                passThrough = operatorController.rightTrigger(0.1);

                ampScore = operatorController.x();

                stow = driverController.b().or(operatorController.a());
                intake = operatorController.leftTrigger(0.1);
                outake = operatorController.leftBumper();
                climb = operatorController.y();
                extendElevator = operatorController.pov(0);
                retractElevator = operatorController.pov(180);
                break;
        }
    }
}
