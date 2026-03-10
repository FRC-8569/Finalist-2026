package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;

public class GlobalConstants {
    public static final Rotation2d BlueAlliance = Rotation2d.k180deg;
    public static final Rotation2d RedAlliance = Rotation2d.kZero;
    public static final Pair<Distance, Distance> CenterLine = Pair.of(Meters.of(8.27052575), Centimeters.of(807.561250).div(2));
    public static final LinearAcceleration G = MetersPerSecondPerSecond.of(9.81);
    public static final AprilTagFieldLayout Field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); 
}
