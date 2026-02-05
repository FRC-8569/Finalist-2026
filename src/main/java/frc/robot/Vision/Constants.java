package frc.robot.Vision;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Constants {
    public static final AprilTagFieldLayout Field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public class FrontCamera {
        public static final String CameraName = "FrontCamera";
        public static final Pose3d CameraPose = new Pose3d(Centimeters.of(6.36217),Centimeters.of(346.01789),Centimeters.of(95.297), new Rotation3d(Rotation2d.kZero));
    }
}
