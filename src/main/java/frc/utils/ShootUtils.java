package frc.utils;

import java.util.Optional;

import org.photonvision.PhotonUtils;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.GlobalConstant;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Shooter.Constants;
import frc.robot.Shooter.Shooter;

public class ShootUtils {
    public static Drivetrain drivetrain = Drivetrain.getInstance();
    public static Shooter shoooter = Shooter.getInstance();
    public static record RobotState(Rotation2d facing, LinearVelocity vel, Angle pitch){};

    public static Optional<RobotState> calculateState(Transform3d target){
        return calculateRobotState(target, ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation()),Constants.Pitch.MaxPitch , Constants.Shoot.MaxVelocity);
    }

    /**
     * calculate the shooter and drivetrain state
     * @param Target the target transform in {@link #Transform3d}
     * @param CurrentSpeeds field relative speeds
     * @param MaximumAngle maximum pitching angle
     * @param MaxVelocity maximum reachable surface velocity of shooter
     * @return
     */
    public static Optional<RobotState> calculateRobotState(Transform3d Target, ChassisSpeeds CurrentSpeeds, Angle MaximumAngle, LinearVelocity MaxVelocity){
        Angle pitch = Radians.of(Math.atan2(Target.getZ(),PhotonUtils.getDistanceToPose(new Pose2d(), new Pose3d(Target.toMatrix()).toPose2d())));
        Rotation2d rawFacing = PhotonUtils.getYawToPose(new Pose2d(), new Pose3d(Target.toMatrix()).toPose2d());
        
        //fuel trajectories
        LinearVelocity vz = MetersPerSecond.of(Math.sqrt(2*GlobalConstant.G.abs(MetersPerSecondPerSecond)*(Target.getZ()+0.5))); //(Target.getZ()+0.5) is the z distance 
        double disc = vz.in(MetersPerSecond)*vz.in(MetersPerSecond) + 2*GlobalConstant.G.in(MetersPerSecondPerSecond)*(Target.getZ()+0.5);
        if(disc < 0.0) return Optional.empty();

        Time t0 = Seconds.of(vz.minus(MetersPerSecond.of(Math.sqrt(disc))).div(GlobalConstant.G).in(MetersPerSecond.per(MetersPerSecondPerSecond))); //actually unit in second bcuz m/s/(m/s^2)
        
        if(t0.lte(Milliseconds.of(1))) return Optional.empty();

        LinearVelocity vPerp = MetersPerSecond.of(-CurrentSpeeds.vxMetersPerSecond*rawFacing.getSin()+CurrentSpeeds.vyMetersPerSecond*rawFacing.getCos());
        Rotation2d psi = Rotation2d.fromRadians(Math.asin(MathUtil.clamp(-vPerp.in(MetersPerSecond)/PhotonUtils.getDistanceToPose(new Pose2d(), new Pose3d(Target.toMatrix()).toPose2d())/t0.in(Seconds), -1.0, 1.0)));

        return Optional.of(new RobotState(rawFacing.plus(psi), MetersPerSecond.of(PhotonUtils.getDistanceToPose(new Pose2d(), new Pose3d(Target.toMatrix()).toPose2d())/t0.in(Seconds)), pitch));
    }   
    
}
