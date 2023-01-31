import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.subsystems.Drive.DriveTrain;
import frc.robot.subsystems.Drive.DriveCal.WheelCal;
import frc.robot.subsystems.Sensors.Odometry;
import frc.robot.subsystems.Sensors.OdometryCals;
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
    void odometryTest(){
        Vector[] wheelLocations = {Vector.fromXY(12.5, -10.75), Vector.fromXY(12.5, 10.75), Vector.fromXY(-12.5, 10.75), Vector.fromXY(-12.5, -10.75)};

        Vector xy = Vector.fromXY(0, 0);
        double zPwr = 1.0;
        Vector[] driveVecs = DriveTrain.formulateDriveVecs(xy, zPwr, 4, wheelLocations);

        driveVecs[0] = new Vector(1.16, 0);
        driveVecs[1] = new Vector(1.15, 0);
      
        System.out.println("Drive Vecs: " + driveVecs[0] + " | " + driveVecs[1] + " | " + driveVecs[2] + " | " + driveVecs[3]);
      
        double[] bestValues = Odometry.formulateBestValues(driveVecs, wheelLocations);
        Vector bestStrafe = new Vector(bestValues[0], bestValues[1]);
        double bestAngle = bestValues[2];
        double error = bestValues[3];

        System.out.println("Best Strafe: " + bestStrafe.toStringXY());
        System.out.println("Best Angle: " + Math.toDegrees(bestAngle));
        System.out.println("Error: " + error);
    }
}