package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.PoseUtils;

public class GlobalConstant {
    public static final Pose3d HubPose = new Pose3d(Centimeters.of(465),Centimeters.of(403),Centimeters.of(175), new Rotation3d(Degrees.of(0),Degrees.of(-90),Degrees.of(0)));
    public static final Rotation2d BlueAlliance = Rotation2d.k180deg;
    public static final Rotation2d RedAlliance = Rotation2d.kZero;
    public static final Pose2d LeftTrench = PoseUtils.getPose(new Pose2d(12,0.6, Rotation2d.k180deg), DriverStation.getAlliance().orElseThrow());
    public static final Pose2d RightTrench = PoseUtils.getPose(new Pose2d(12,7.5, Rotation2d.k180deg), DriverStation.getAlliance().orElseThrow());
    public static final LinearAcceleration G = MetersPerSecondPerSecond.of(-9.81);
    
}
