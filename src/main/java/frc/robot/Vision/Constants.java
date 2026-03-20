package frc.robot.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Constants {
    public class RearCamera {
        public static final String CameraName = "BackCamera";
        public static final Transform3d CameraPlace = new Transform3d(Millimeters.of(103.74417), Millimeters.of(243.42749), Millimeters.of(417.99980), new Rotation3d(Degrees.zero(), Degrees.of(15), Degrees.of(30)));
        
    }
}