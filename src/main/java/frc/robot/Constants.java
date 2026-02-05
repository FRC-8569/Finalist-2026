package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Constants {
    public static final Pose3d HubPose = new Pose3d(Centimeters.of(465),Centimeters.of(403),Centimeters.of(175), new Rotation3d(Degrees.of(0),Degrees.of(-90),Degrees.of(0)));

}
