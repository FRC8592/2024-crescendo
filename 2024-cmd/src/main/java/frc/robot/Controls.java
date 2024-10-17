package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CONTROLLERS;

public final class Controls {
    private static final CommandXboxController driverController = new CommandXboxController(
        CONTROLLERS.DRIVER_PORT
    );
    private static final CommandXboxController operatorController = new CommandXboxController(
        CONTROLLERS.OPERATOR_PORT
    );

    /**
     * Enum for the different sets of controls (different drivers,
     * different DS configurations, etc)
     */
    protected enum ControlSets{
        MAIN_TELEOP,
        SINGLE_CONTROLLER,
        SINGLE_CONTROLLER_OTHER_DRIVER,
        DISABLED,
    }

    protected static DoubleSupplier driveTranslateX = () -> 0;
    protected static DoubleSupplier driveTranslateY = () -> 0;
    protected static DoubleSupplier driveRotate = () -> 0;

    protected static Trigger slowMode = new Trigger(() -> false);
    protected static Trigger zeroGryoscope = new Trigger(() -> false);

    protected static Trigger autocollect = new Trigger(() -> false);
    protected static Trigger visionDemo = new Trigger(() -> false);

    protected static Trigger snapForward = new Trigger(() -> false);
    protected static Trigger snapBack = new Trigger(() -> false);
    protected static Trigger snapLeft = new Trigger(() -> false);
    protected static Trigger snapRight = new Trigger(() -> false);

    protected static Trigger partyMode = new Trigger(() -> false);

    protected static Trigger visionShoot = new Trigger(() -> false);
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
     * Change the variables in the Controls class to match the specified
     * control set. Note that this doesn't edit or remove bindings.
     *
     * @param set the control set to apply
     */
    protected static void applyControlSet(ControlSets set){
        switch (set) {
            default: case DISABLED:
                driveTranslateX = () -> 0;
                driveTranslateY = () -> 0;
                driveRotate = () -> 0;

                slowMode = new Trigger(() -> false);
                zeroGryoscope = new Trigger(() -> false);

                autocollect = new Trigger(() -> false);
                visionDemo = new Trigger(() -> false);

                snapForward = new Trigger(() -> false);
                snapBack = new Trigger(() -> false);
                snapLeft = new Trigger(() -> false);
                snapRight = new Trigger(() -> false);

                partyMode = new Trigger(() -> false);

                visionShoot = new Trigger(() -> false);
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
                driveTranslateX = () -> -driverController.getLeftX();
                driveTranslateY = () -> -driverController.getLeftY();
                driveRotate = () -> -driverController.getRightX();

                slowMode = driverController.rightBumper();
                zeroGryoscope = driverController.back();

                autocollect = driverController.a();

                snapForward = driverController.pov(0);
                snapBack = driverController.pov(180);
                snapLeft = driverController.pov(90);
                snapRight = driverController.pov(270);


                partyMode = operatorController.start();
                passThrough = operatorController.back();
                visionShoot = operatorController.rightBumper();
                score = operatorController.rightTrigger(0.1);

                ampScore = operatorController.x();

                stow = operatorController.a();
                intake = operatorController.leftTrigger(0.1);
                climb = operatorController.y();
                extendElevator = operatorController.pov(0);
                retractElevator = operatorController.pov(180);
                break;

            case SINGLE_CONTROLLER:
                driveTranslateX = () -> -driverController.getLeftX();
                driveTranslateY = () -> -driverController.getLeftY();
                driveRotate = () -> -driverController.getRightX();

                slowMode = driverController.rightBumper();
                zeroGryoscope = driverController.back();

                autocollect = driverController.a();
                visionDemo = driverController.leftTrigger(0.1);

                snapForward = driverController.pov(0);
                snapBack = driverController.pov(180);
                snapLeft = driverController.pov(90);
                snapRight = driverController.pov(270);

                partyMode = driverController.start();

                visionShoot = operatorController.rightBumper();
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

            case SINGLE_CONTROLLER_OTHER_DRIVER: // Same controls as single controller, but with the other controllers used to "allow" certain controls
                driveTranslateX = () -> -driverController.getLeftX();
                driveTranslateY = () -> -driverController.getLeftY();
                driveRotate = () -> -driverController.getRightX();

                slowMode = driverController.rightBumper();
                zeroGryoscope = driverController.back();

                autocollect = driverController.a();
                visionDemo = driverController.leftTrigger(0.1);

                snapForward = driverController.pov(0);
                snapBack = driverController.pov(180);
                snapLeft = driverController.pov(90);
                snapRight = driverController.pov(270);

                partyMode = driverController.start();

                visionShoot = operatorController.rightBumper();
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
