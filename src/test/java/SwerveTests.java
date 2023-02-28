import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.subsystems.Drive.DriveTrain;
import frc.robot.subsystems.Sensors.Odometry;
import frc.robot.util.Util;
import frc.robot.util.Vector;

public class SwerveTests {

    static final double DELTA = 1e-6;
    //Odometry odometry;

    @BeforeEach
    void setup(){
        //OdometryCals cals = new OdometryCals();
        //cals.maxStandardDeviations = 1.7;
        //WheelCal[] wCal = new WheelCal[4];
        //odometry = new Odometry(cals, wCal);
    }

    @AfterEach
    void shutdown() throws Exception {
        
    }

    @Test
    void vectorAddition(){
        Vector v = Vector.fromXY(-3, 10);
        v.add(Vector.fromXY(2, -3));

        //System.out.println(v.getX() + " " + v.getY());
        assertEquals(-1, v.getX(), DELTA);
        assertEquals(7, v.getY(), DELTA);
        
    }

    @Test
    void vectorAverage(){
        Vector[] v = new Vector[2];
        v[0] = Vector.fromXY(3, 4);
        v[1] = Vector.fromXY(7, 8);

        //Vector expected = Vector.fromXY(6, 8);
        //Vector result = Vector.averageVectors(v);
    }

    @Test
    void odometryTest(){/*
        Vector[] wheelLocations = {Vector.fromXY(12.5, -10.75), Vector.fromXY(12.5, 10.75), Vector.fromXY(-12.5, 10.75), Vector.fromXY(-12.5, -10.75)};

        Vector xy = Vector.fromXY(0.5, 0.0);
        double zPwr = 0.0;
        Vector[] driveVecs = DriveTrain.formulateDriveVecs(xy, zPwr, 4, wheelLocations);

        //driveVecs[0] = new Vector(1.16, 0);
        //driveVecs[1] = new Vector(1.15, 0);
      
        System.out.println("Drive Vecs: " + driveVecs[0].toStringXY() + " | " + driveVecs[1].toStringXY() + " | " + driveVecs[2].toStringXY() + " | " + driveVecs[3].toStringXY());
      
        double[] bestValues = Odometry.formulateOkValues(driveVecs, wheelLocations);
        Vector bestStrafe = new Vector(bestValues[0], bestValues[1]);
        double bestAngle = bestValues[2];
        double error = bestValues[3];

        System.out.println("Best Strafe: " + bestStrafe.toStringXY());
        System.out.println("Best Angle: " + Math.toDegrees(bestAngle));
        System.out.println("Error: " + error);
        System.out.println("Best Group: " + bestValues[4]);*/
    }

    @Test
    void odoCenterOfRotTest(){
        Vector[] wheelLocations = {Vector.fromXY(12.5, -10.75), Vector.fromXY(12.5, 10.75), Vector.fromXY(-12.5, 10.75), Vector.fromXY(-12.5, -10.75)};

        Vector xy = Vector.fromXY(0.5, 0.0);
        double zPwr = 0.0;
        Vector[] driveVecs = DriveTrain.formulateDriveVecs(xy, zPwr, 4, wheelLocations);

        for (Vector vector : driveVecs) {
            vector.r *= 10;
        }
        //driveVecs[0].r = 5.0;
        //driveVecs[1].r = 99;
        //driveVecs[2].r = 150;
        //driveVecs[3].r = 5.0;

        //Vector[][] centersOfRot = Odometry.formulateCentersOfRot(driveVecs, wheelLocations);

        //System.out.println("FR Centers Of Rot: " + centersOfRot[0][0] + " : " + centersOfRot[0][1] + " : " + centersOfRot[0][2]);
        //System.out.println("FL Centers Of Rot: " + centersOfRot[1][0] + " : " + centersOfRot[1][1] + " : " + centersOfRot[1][2]);
        //System.out.println("RL Centers Of Rot: " + centersOfRot[2][0] + " : " + centersOfRot[2][1] + " : " + centersOfRot[2][2]);
        //System.out.println("RR Centers Of Rot: " + centersOfRot[3][0] + " : " + centersOfRot[3][1] + " : " + centersOfRot[3][2]);

        double[] vals = Odometry.formulateBestValues(driveVecs, wheelLocations);
        Vector strafe = new Vector(vals[0], vals[1]);
        double angle = vals[2];
        double error = vals[3];

        System.out.println("Strafe: " + strafe);
        System.out.println("Angle: " + angle);
        System.out.println("Error " + error);
    }

    @Test
    void interpTest(){
        double[] a = {0,10,22};
        double[] b = {10,-10,10};
        double v = Util.interp(a,b,-1);
        assertEquals(10,v,DELTA);
        v = Util.interp(a,b,5);
        assertEquals(0,v,DELTA);
        v = Util.interp(a,b,23);
        assertEquals(10,v,DELTA);
        v = Util.interp(a,b,21);
        assertEquals(25 / 3.0,v,DELTA);

    }
}