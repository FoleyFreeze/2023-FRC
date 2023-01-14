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
    void normalizeTest(){
        
<<<<<<< HEAD
        //assertEquals();
=======
        System.out.println("tgtAng: " + tgtAng + "   r: " + r);
>>>>>>> e1655b29cc68c35a2a03e1471a99cc7d139746a8
    }

}