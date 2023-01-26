package frc.robot.subsystems.Sensors;

public class OdometryCals {
    public boolean averageWheelAng = false;//Keep this as false, I don't think this works anymore
    public double maxStandardDeviations = 1.7;//lowering this decreases the threshold of wheels allowed to pass through

    public double maxNavXWheelAngDiff = 0.017;//1 degree off
}
