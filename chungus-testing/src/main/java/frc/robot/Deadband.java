package frc.robot;

public class Deadband {
    public static double joystickDeadband(double input){
        if (Math.abs(input) < Constants.CONTROLLER_JOYSTICK_DEADBAND) {
            return 0;
        } else {
            return input;
        }
    }
}
