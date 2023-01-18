import static org.junit.jupiter.api.Assertions.assertEquals;

import java.beans.Transient;

import org.junit.jupiter.api.Test;

import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class SwerveTests {

    static final double DELTA = 1e-6;

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

}