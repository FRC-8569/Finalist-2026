package frc.utils;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;

import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.GlobalConstants;
import frc.robot.Drivetrain.Drivetrain;

public class Tools {
    public static record RobotState(Rotation2d drivetrainFacing, SwerveModuleState shooterState) implements Subsystem {
        private static final double g = 9.81;
        private static double distance = -1, height = -1;

        public Rotation2d getFacing() {
            return drivetrainFacing;
        }

        public SwerveModuleState getShootingState() {
            return shooterState;
        }

        /**
         * This function creates a basic set of shooting profile, recommend to use
         * {@link #optmize()} to optmize the shooting profile
         * 
         * @return
         */
        public static RobotState create(Transform3d target) {
            Rotation2d facing = new Rotation2d(target.getMeasureX().in(Meters), target.getMeasureY().in(Meters));
            
            double d = Math.hypot(target.getMeasureX().in(Meters), target.getMeasureY().in(Meters));
            double h = target.getMeasureZ().plus(Meters.of(5)).in(Meters);

            Rotation2d pitch = Rotation2d.fromRadians(Math.atan2(h, d));
            double v = Math.sqrt(2*g*height)/Math.abs(pitch.getSin());

            distance = d;
            height = h;

            DogLog.log("ShootPredictor/RobotFacing", facing);
            DogLog.log("ShootPredictor/ShooterState", new SwerveModuleState(MetersPerSecond.of(v), pitch));
            DogLog.log("ShootPredictor/Target", new Pose3d(Drivetrain.getInstance().getState().Pose).plus(target));
            DogLog.log("ShootPredictor/TimeStamp", Utils.getCurrentTimeSeconds());

            return new RobotState(facing, new SwerveModuleState(MetersPerSecond.of(v), pitch));
        }

        @SuppressWarnings("unused")
        private Angle calcAngle(LinearVelocity endVelocity){
            return Rotation2d.fromRadians(Math.asin(g*distance/(endVelocity.in(MetersPerSecond)*endVelocity.in(MetersPerSecond)))).div(2).getMeasure();
        }

        @SuppressWarnings("unused")
        private LinearVelocity calcVel(Angle a){
            return MetersPerSecond.of(Math.sqrt(2*g*height)/Math.abs(new Rotation2d(a).getSin()));
        }

        public static RobotState create(FieldObjects obj) {
            return create(new Transform3d(new Pose3d(Drivetrain.getInstance().getState().Pose), obj.getPose()));
        }
    }

    public static record ShootPreset(Pose2d pose, RobotState state){
        public Pose2d getPose(){
            return new Pose2d(pose.getX(),pose.getY(),state.drivetrainFacing);
        }

        public SwerveModuleState getShooterState(){
            return state.shooterState;
        }
    }

    private static Pose2d getPose(Pose2d raw, Alliance alliance, FieldSide side){
        Distance dX = GlobalConstants.CenterLine.getFirst().minus(raw.getMeasureX());
        Distance dY = GlobalConstants.CenterLine.getSecond().minus(raw.getMeasureY());

        boolean shouldMirrorX = alliance == Alliance.Blue ? dX.lt(Meters.zero()) : dX.gt(Meters.zero());
        boolean shouldMirrorY =
                    side != null &&
                    (alliance == Alliance.Blue
                        ? (side == FieldSide.LEFT ? dY.lt(Meters.zero()) : dY.gt(Meters.zero()))
                        : (side == FieldSide.LEFT ? dY.gt(Meters.zero()) : dX.lt(Meters.zero())));

        return new Pose2d(
            shouldMirrorX ? GlobalConstants.CenterLine.getFirst().plus(dX) : raw.getMeasureX(),
            shouldMirrorY ? GlobalConstants.CenterLine.getSecond().plus(dY) : raw.getMeasureY() , 
            shouldMirrorX ? new Rotation2d(Degrees.of(180).minus(raw.getRotation().getMeasure())) : raw.getRotation());

    }

    public static Pose2d getPose(Pose2d raw, FieldSide side){
        return getPose(raw, DriverStation.getAlliance().orElse(Alliance.Blue), side);
    }

    public static Pose3d getPose(Pose3d raw, FieldSide side){
        Pose2d edited = getPose(raw.toPose2d(), side);
        return new Pose3d(edited.getX(),edited.getY(), raw.getZ(), new Rotation3d(raw.getRotation().getMeasureX(), raw.getRotation().getMeasureY(), edited.getRotation().getMeasure()));
    }

    public static enum FieldSide{
        LEFT, RIGHT;
    }

    
    public static final FieldObjects HUB = new FieldObjects(new Pose3d(Centimeters.of(465),Centimeters.of(403),Centimeters.of(175), new Rotation3d(Degrees.of(0),Degrees.of(-90),Degrees.of(0))));
    public static final FieldObjects Tower = new FieldObjects(new Pose3d(new Pose2d(0.9, 4.65, Rotation2d.k180deg)));
    public static FieldObjects CurrentAlliance = new FieldObjects(new Pose3d());
    public record FieldObjects(Pose3d pose) {
        public Pose3d getPose(){
            return Tools.getPose(pose, null);
        }
    }

    public enum FieldPlace{
        BlueZone(4.652613,0),
        RedZone(16.541052,11.942413),
        Neutral(11.942413,4.652613);

        Distance Start, End;
        FieldPlace(double start, double end){
            Start = Meters.of(start);
            End = Meters.of(end);
        }
         public Pair<Distance, Distance> getValue(){
            return Pair.of(Start, End);
        }

        public boolean contains(Pose2d pose){
            return pose.getMeasureX().lte(Start) && pose.getMeasureX().gt(End);
        }
    }
}
