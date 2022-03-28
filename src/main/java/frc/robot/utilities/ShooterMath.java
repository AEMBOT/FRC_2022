package frc.robot.utilities;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import java.util.Map;
import java.util.TreeMap;

/** Used for calculations of shooter flywheel RPM based on Limelight readings. */
public class ShooterMath {
  // Keys: Limelight y angles, Values: RPMs
  // These were found through testing
  private static final TreeMap<Double, Double> m_rpmMap =
      new TreeMap<>(
          Map.ofEntries(
              Map.entry(13.3, 2468.0),
              Map.entry(9.0, 2680.0),
              Map.entry(3.0, 2960.0),
              Map.entry(-0.2, 3205.0),
              Map.entry(-6.0, 3900.0)));

  /**
   * Returns the interpolated RPM based on experimentally determined values.
   * @param yAngle The y angle obtained from the Limelight
   * @return The calculated shooter RPM
   */
  public static double calculateRPM(double yAngle) {
    Double keyAbove = m_rpmMap.ceilingKey(yAngle);
    Double keyBelow = m_rpmMap.floorKey(yAngle);

    // Return the default RPM if the angle is outside of the pre-sampled range
    if (keyAbove == null || keyBelow == null) {
      return calculateRPM(kDefaultYAngle);
    }

    // Otherwise interpolate between the keys surrounding yAngle
    else {
      // distanceBetween has to be on the interval [0, 1]
      double distanceBetween = (yAngle - keyBelow) / (keyAbove - keyBelow);
      return MathUtil.interpolate(m_rpmMap.get(keyBelow), m_rpmMap.get(keyAbove), distanceBetween);
    }
  }
}
