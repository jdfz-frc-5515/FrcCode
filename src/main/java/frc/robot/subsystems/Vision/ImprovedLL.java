package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class ImprovedLL extends LimelightHelpers{
    public static class MT2stddevs{
        public double xdev = 0.;
        public double ydev = 0.;
        public MT2stddevs(double x, double y){
            xdev = x;
            ydev = y;
        }
    }

    public static MT2stddevs getmt2Devs(){
        Double[] devs = NetworkTableInstance.getDefault().getTable(Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT).getEntry("stddevs").getDoubleArray(new Double[12]);
        return new MT2stddevs(devs[6], devs[7]);
    }
}
