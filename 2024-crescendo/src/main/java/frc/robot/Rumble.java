package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public final class Rumble {
    private static ArrayList<RumbleBump> driverQueue = new ArrayList<RumbleBump>();;
    private static ArrayList<RumbleBump> operatorQueue = new ArrayList<RumbleBump>();;
    private static Timer driverTimer = new Timer();
    private static Timer operatorTimer = new Timer();
    public static void init(){
        driverTimer.reset();
        driverTimer.start();
    
        operatorTimer.reset();
        operatorTimer.start();
    }

    public static void update(XboxController driver, XboxController operator){
        updateDriver(driver);
        updateOperator(operator);
    }

    public static void clearQueue(Controller controller){
        switch(controller){
            case DRIVER:
                driverQueue.clear();
                break;
            case OPERATOR:
                operatorQueue.clear();
                break;
        }
    }

    public static void enqueueRumbleBump(Controller controller, RumbleBump bump){
        if(!DriverStation.isAutonomous()){
            switch(controller){
                case DRIVER:
                    driverQueue.add(bump);
                    break;
                case OPERATOR:
                    operatorQueue.add(bump);
                    break;
            }
        }
    }

    private static void updateDriver(XboxController driver){
        if(!driverQueue.isEmpty()){ // Note the exclamation point
            RumbleBump currentBump = driverQueue.get(0);
            if(!driverTimer.hasElapsed(currentBump.duration)){ // Note the exclamation point again
                // driver.setRumble(RumbleType.kBothRumble, (int)(currentBump.strength*255));
            }
            else{
                driverQueue.remove(currentBump);
                driverTimer.reset();
                driverTimer.start();
            }
        }
        else{
            driverTimer.reset();
            driver.setRumble(RumbleType.kBothRumble, 0);
        }
    }
    private static void updateOperator(XboxController operator){
        if(!operatorQueue.isEmpty()){
            RumbleBump currentBump = operatorQueue.get(0);
            if(!operatorTimer.hasElapsed(operatorQueue.get(0).duration)){
                // operator.setRumble(RumbleType.kBothRumble, (int)(currentBump.strength*255));
            }
            else{
                operatorQueue.remove(0);
                operatorTimer.reset();
                operatorTimer.start();
            }
        }
        else{
            operatorTimer.reset();
        }
    }

    public static boolean isQueueEmpty(Controller controller){
        switch(controller){
            case DRIVER:
                return driverQueue.isEmpty();
            case OPERATOR:
                return operatorQueue.isEmpty();
            default: // Never reached, but Java requires this to be here to compile
                return true;
        }
    }

    public class RumbleBump{
        public double duration;
        public double strength;
        /**
         * Create a new delay between rumbles for the specified controller
         * @param duration the time in seconds (decimals supported) to run the rumble
         */
        public RumbleBump(double duration){
            this.duration = duration;
            this.strength = 0;
        }

        /**
         * Create a new rumble bump
         * @param duration the time in seconds (decimals supported) to run the rumble
         * @param strength the power to rumble at (0-1)
         */
        public RumbleBump(double duration, double strength){
            this.duration = duration;
            this.strength = strength;
        }
    }
    public enum Controller{
        DRIVER,
        OPERATOR
    }
}
