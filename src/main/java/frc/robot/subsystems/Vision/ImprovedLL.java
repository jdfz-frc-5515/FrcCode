package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class ImprovedLL extends LimelightHelpers{
    public static class MT2stddevs{
        public double xdev = 0.;
        public double ydev = 0.;
        public MT2stddevs(double x, double y){
            xdev = x;
            ydev = y;
        }
    }

    public static MT2stddevs getmt2Devs(String llName){
        Double[] devs = NetworkTableInstance.getDefault().getTable(llName).getEntry("stddevs").getDoubleArray(new Double[12]);
        return new MT2stddevs(devs[6], devs[7]);
    }
}
