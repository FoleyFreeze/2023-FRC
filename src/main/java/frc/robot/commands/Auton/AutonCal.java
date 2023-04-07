package frc.robot.commands.Auton;

public class AutonCal {

    public static class MPCals {
        public double maxAccel;
        public double maxVel;
        public double kP_MP;
        public double kA;
        public double kV;
        public double kS;
    }

    public static MPCals driveBase = new MPCals();
    static {
        driveBase.maxAccel = 120;
        driveBase.maxVel = 120;
        driveBase.kP_MP = 5.0;
        driveBase.kA = 0.001;
        driveBase.kV = 0.006;
        driveBase.kS = 0.03;
    }

    public static MPCals scoreBase = new MPCals();
    static {
        scoreBase.maxAccel = 90;
        scoreBase.maxVel = 120;
        scoreBase.kP_MP = 5.0;
        scoreBase.kA = 0.001;
        scoreBase.kV = 0.006;
        scoreBase.kS = 0.03;
    }

    public static MPCals angleBase = new MPCals();
    static {
        angleBase.maxAccel = 80;
        angleBase.maxVel = 60;
        angleBase.kP_MP = 7.0;
        angleBase.kA = 0.0007;
        angleBase.kV = 0.006;
        angleBase.kS = 0.03;
    }

    public static MPCals bumpThereCals = new MPCals();
    static {
        bumpThereCals.maxAccel = 140;
        bumpThereCals.maxVel = 55;
        bumpThereCals.kP_MP = 3.0;
        bumpThereCals.kA = 0.0005;
        bumpThereCals.kV = 0.006;
        bumpThereCals.kS = 0.03;
    }

    public static MPCals bumpBackCals = new MPCals();
    static {
        bumpBackCals.maxAccel = 140;
        bumpBackCals.maxVel = 65;
        bumpBackCals.kP_MP = 3.0;
        bumpBackCals.kA = 0.0005;
        bumpBackCals.kV = 0.006;
        bumpBackCals.kS = 0.03;
    }

}
