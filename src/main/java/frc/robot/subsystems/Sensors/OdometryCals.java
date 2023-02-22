package frc.robot.subsystems.Sensors;

public class OdometryCals {
    public boolean averageWheelAng = false;//Keep this as false, I don't think this works anymore
    public double maxStandardDeviations = 1.7;//lowering this decreases the threshold of wheels allowed to pass through

    public double maxNavXWheelAngDiff = 0.017;//1 degree off

    public static double testDValue = 0.001;
    public static double maxStandardDeviationsStrafeCOR = 1.0;
    public static double maxStandardDeviationsAngleCOR = 1.0;
}
