import static org.junit.jupiter.api.Assertions.assertEquals;

import java.beans.Transient;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.subsystems.Drive.DriveCal;
import frc.robot.subsystems.Drive.DriveCal.WheelCal;
import frc.robot.subsystems.Sensors.Odometry;
import frc.robot.subsystems.Sensors.OdometryCals;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class SwerveTests {

    static final double DELTA = 1e-6;
    Odometry odometry;

    @BeforeEach
    void setup(){
        OdometryCals cals = new OdometryCals();
        //cals.maxStandardDeviations = 1.7;
        WheelCal[] wCal = new WheelCal[4];
        odometry = new Odometry(cals, wCal);
    }

    @AfterEach
    void shutdown() throws Exception {
        odometry.close();
    }

    @Test
    void vectorAddition(){
        Vector v = Vector.fromXY(-3, 10);
        v.add(Vector.fromXY(2, -3));

        System.out.println(v.getX() + " " + v.getY());
        assertEquals(-1, v.getX(), DELTA);
        assertEquals(7, v.getY(), DELTA);
        
    }

    @Test
    void vectorAverage(){
        Vector[] v = new Vector[2];
        v[0] = Vector.fromXY(3, 4);
        v[1] = Vector.fromXY(7, 8);

        Vector expected = Vector.fromXY(6, 8);
        Vector result = Vector.averageVectors(v);
    }

    @Test
    void odometryTest(){
        Vector[] v = {Vector.fromXY(5, 5), Vector.fromXY(5, 5), Vector.fromXY(5, 5), Vector.fromXY(5.1, 5)};

        Vector[] result = odometry.checkVCriteria(v);
        System.out.println("result: " + result[0] + " | " + result[1] + " | " + result[2] + " | " + result[3]);
    }
}