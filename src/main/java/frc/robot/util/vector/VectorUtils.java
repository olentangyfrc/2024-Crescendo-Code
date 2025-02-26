package frc.robot.util.vector;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

public class VectorUtils {
    public static Translation2d vecToTranslation2d(Vector<N2> vec) {
        return new Translation2d(vec.get(0, 0), vec.get(0, 1));
    }

    public static Vector<N2> translation2dToVec(Translation2d translation2d) {
        return VecBuilder.fill(translation2d.getX(), translation2d.getY());
    }
}
