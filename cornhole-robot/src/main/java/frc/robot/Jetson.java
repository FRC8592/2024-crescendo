package frc.robot;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Jetson {
    private NetworkTableInstance nt;
    private NetworkTable table;

    private DoublePublisher distancePub;
    private DoublePublisher xPub;
    private DoublePublisher yPub;
    private BooleanPublisher alivePub;
    

    private DoubleSubscriber distanceSub;
    private DoubleSubscriber xSub;
    private DoubleSubscriber ySub;
    private BooleanSubscriber aliveSub;

    public Jetson() {
        nt = NetworkTableInstance.getDefault();

        table = nt.getTable("jetson");

        distancePub = table.getDoubleTopic("distance").publish();
        distanceSub = table.getDoubleTopic("distance").subscribe(0.0);

        xPub = table.getDoubleTopic("x").publish();
        xSub = table.getDoubleTopic("x").subscribe(0.0);

        yPub = table.getDoubleTopic("y").publish();
        ySub = table.getDoubleTopic("y").subscribe(0.0);
        
        alivePub = table.getBooleanTopic("alive").publish();
        aliveSub = table.getBooleanTopic("alive").subscribe(false);

    }

    public double getDistance(){
        return distanceSub.get();
    }
    public double getX(){
        return xSub.get();
    }
    public double getY(){
        return ySub.get();
    }

    public boolean isAlive(){
        return aliveSub.get();
    }
}
