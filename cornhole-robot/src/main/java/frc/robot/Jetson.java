package frc.robot;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Jetson {
    private NetworkTableInstance nt;
    private NetworkTable table;
    private DoublePublisher xPub;
    private DoublePublisher yPub;
    private DoubleSubscriber xSub;
    private DoubleSubscriber ySub;

    public Jetson() {
        nt = NetworkTableInstance.getDefault();

        table = nt.getTable("jetson");

        xPub = table.getDoubleTopic("x").publish();
        yPub = table.getDoubleTopic("y").publish();

        xSub = table.getDoubleTopic("x").subscribe(0.0);
        ySub = table.getDoubleTopic("y").subscribe(0.0);

    }

    public double getX(){
        return xSub.get();
    }

    public double getY(){
        return ySub.get();
    }
}
