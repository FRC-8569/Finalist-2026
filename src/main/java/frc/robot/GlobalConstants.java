package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class GlobalConstants {
    public static final Distance CenterLineX = Meters.of(8.27052575);
    public static final Distance CenterLineY = Meters.of(8.072088).div(2);

    public static final double KrakenX44FOCSpeed = 122.8; // in MPS
    public static final double KrakenX60FOCSpeed = 5800.0/60;
}
