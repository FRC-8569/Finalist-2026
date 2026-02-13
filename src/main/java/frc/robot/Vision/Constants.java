package frc.robot.Vision;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Constants {
    public static final AprilTagFieldLayout Field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public class FrontCamera {
        public static final String CameraName = "FrontCamera";
        public static final Pose3d CameraPose = new Pose3d(Centimeters.of(25.876551),Centimeters.of(19.550258),Centimeters.of(29.575464), new Rotation3d(Degrees.of(0), Degrees.of(-30), Degrees.of(60)));
    }
}
