package frc.utils;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.GlobalConstants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Shooter.Constants;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.Constants.Pitch;
import frc.robot.Shooter.Constants.Shoot;

public class Tools {
    public static RobotState getCurrentState(){
        return new RobotState(Drivetrain.getInstance().getState().Pose.getRotation(), Shooter.getInstance().getState());
    }
   
    public static RobotState getTargetState(){
        return new RobotState(Drivetrain.getInstance().ManualFacing.TargetDirection, new SwerveModuleState(Shooter.getInstance().ShootPID.getVelocityMeasure().in(RadiansPerSecond)*Shoot.WheelRadius.in(Meters),new Rotation2d(Shooter.getInstance().PitchingPID.getPositionMeasure())));
    }
   
    public static RobotState getPredictState(){
        return RobotState.create(Tools.HUB, Drivetrain.getInstance().getState().Pose);
    }

    public static record RobotState(Rotation2d drivetrainFacing, SwerveModuleState shooterState) implements Subsystem {
        private static final double g = 9.81;
        private static double height = -1;

        public Rotation2d getFacing() {
            return drivetrainFacing;
        }

        public SwerveModuleState getShootingState() {
            return shooterState;
        }

        public boolean isNear(RobotState other){
            return drivetrainFacing.getMeasure().isNear(Drivetrain.getInstance().getState().Pose.getRotation().getMeasure(), 0.05)
                && MetersPerSecond.of(shooterState.speedMetersPerSecond).isNear(MetersPerSecond.of(other.shooterState.speedMetersPerSecond), 0.1) &&
                shooterState.angle.getMeasure().isNear(other.shooterState.angle.getMeasure(), 0.05);
        }

        private LinearVelocity calcVelocity(Angle angle){
            return MetersPerSecond.of(Math.sqrt(2*g*height)/Math.abs(new Rotation2d(angle).getSin()));
        }

        /**
         * This function creates a basic set of shooting profile, recommend to use
         * {@link #optmize()} to optmize the shooting profile
         * 
         * @return
         */
        public static RobotState create(Transform3d target) {
            Rotation2d facing = Rotation2d.fromRadians(Math.atan2(target.getMeasureY().in(Meters), target.getMeasureX().in(Meters)));
            facing = new Rotation2d(facing.getMeasure());

            double d = Math.hypot(target.getMeasureX().in(Meters), target.getMeasureY().in(Meters));
            double h = target.getMeasureZ().plus(Meters.of(2)).in(Meters);

            Rotation2d pitch = Rotation2d.fromRadians(Math.atan2(h, d));
            double v = Math.sqrt(2*g*height)/Math.abs(pitch.getSin());
            height = h;

            DogLog.log("ShootPredictor/RobotFacing", facing);
            DogLog.log("ShootPredictor/ShooterState", new SwerveModuleState(MetersPerSecond.of(v), pitch));
            DogLog.log("ShootPredictor/Target", new Pose3d(Drivetrain.getInstance().getState().Pose).plus(Constants.ShooterPlace).plus(target));
            DogLog.log("ShootPredictor/TimeStamp", Utils.getCurrentTimeSeconds());

            return new RobotState(facing.plus(new Rotation2d(Degrees.of(225))), new SwerveModuleState(MetersPerSecond.of(v), pitch));
        }

        public static RobotState getNeutralState(){
            double d = Math.abs(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? Drivetrain.getInstance().getState().Pose.getMeasureX().minus(Meters.of(14)).in(Meters) : Drivetrain.getInstance().getState().Pose.getMeasureX().minus(Meters.of(2)).in(Meters));
            return new RobotState(Rotation2d.k180deg, new SwerveModuleState(Math.sqrt(g*d*d/(d-4))+5, Rotation2d.fromDegrees(45)));
        }

        public static Rotation2d getDrivetrainFacing(Transform3d target){
            return Rotation2d.fromRadians(Math.atan2(target.getY(), target.getX()));
        }

        public static Rotation2d getDrivetrainFacing(FieldObjects obj, Pose2d target){
            return getDrivetrainFacing(new Transform3d(new Pose3d(target), obj.getPose()));
        }
        
        public static RobotState create(FieldObjects obj, Pose2d targetPose) {
            return create(new Transform3d(new Pose3d(targetPose), obj.getPose()));
        }

        public RobotState optmize(){
            //fit the constraints
            LinearVelocity calcedVel = MetersPerSecond.zero();
            Rotation2d calcedAngle = drivetrainFacing;
            if(shooterState.angle.getMeasure().lt(Pitch.PitchConstraints.getFirst())){
                calcedVel = calcVelocity(Pitch.PitchConstraints.getFirst());
            }else if(shooterState.angle.getMeasure().gt(Pitch.PitchConstraints.getSecond())){
                calcedVel = calcVelocity(Pitch.PitchConstraints.getSecond());
            }

            DogLog.log("ShootPredictor/OptmizedFacing", calcedAngle);
            DogLog.log("ShootPredictor/OptmizedState", new SwerveModuleState(calcedVel, shooterState.angle));

            return new RobotState(drivetrainFacing, new SwerveModuleState(calcedVel, shooterState.angle));
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
            shouldMirrorX ? Rotation2d.k180deg.minus(raw.getRotation().times(shouldMirrorY ? 2 : 1)) : raw.getRotation());

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

        public static FieldSide getSide(Pose2d pose) {
            Distance dy = pose.getMeasureY().minus(GlobalConstants.CenterLine.getSecond());
            return ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? dy.gt(Meters.zero()) : dy.lt(Meters.zero())) ? FieldSide.LEFT : FieldSide.RIGHT;
        }
    }

    
    public static final FieldObjects HUB = new FieldObjects(new Pose3d(Centimeters.of(465),Centimeters.of(403),Centimeters.of(175), new Rotation3d(Degrees.of(0),Degrees.of(-90),Degrees.of(0))));
    public static final FieldObjects Tower = new FieldObjects(new Pose3d(new Pose2d(0.9, 4.65, Rotation2d.k180deg)));
    public static FieldObjects CurrentAlliance = new FieldObjects(new Pose3d());
    public record FieldObjects(Pose3d pose) {
        public Pose3d getPose(){
            return Tools.getPose(pose, null);
        }
    }


    public static record ShootPreset(Pose2d pose, RobotState state){
        public Pose2d getPose(){
            return new Pose2d(pose.getX(), pose.getY(), state.drivetrainFacing);
        }

        public ShootPreset getMirrorPose(){
            Distance dY = GlobalConstants.CenterLine.getSecond().minus(pose.getMeasureY());
            
            return new ShootPreset(new Pose2d(pose.getMeasureX(), GlobalConstants.CenterLine.getSecond().plus(dY), Rotation2d.k180deg), new RobotState(Rotation2d.kCW_90deg.minus(state.getFacing().unaryMinus()), getShooterState()));
        }

        public SwerveModuleState getShooterState(){
            return state.shooterState;
        }

        public double getDistance(){
            Transform2d t = new Transform2d(Drivetrain.getInstance().getState().Pose, Tools.getPose(pose, null));
            return Math.hypot(t.getMeasureX().in(Meters), t.getMeasureY().in(Meters));
        }
    }

    public static ShootPreset RightBump = new ShootPreset(new Pose2d(14.5,6.2, Rotation2d.kZero), new RobotState(Rotation2d.fromDegrees(-166.88), new SwerveModuleState(17, Rotation2d.fromDegrees(65))));
    public static ShootPreset LeftBump = new ShootPreset(new Pose2d(14.5,1.3, Rotation2d.kZero), new RobotState(Rotation2d.fromDegrees(13.12-90), new SwerveModuleState(16, Rotation2d.fromDegrees(65))));

    public static boolean isShootable(){
        return isHubActive() && Drivetrain.getInstance().inZone();
    }

    public static boolean isHubActive(){
        return switch(DriverStation.getGameSpecificMessage()){
            case "R" -> Alliance.Red;
            case "B" -> Alliance.Blue;
            default -> Alliance.Blue;
        } == DriverStation.getAlliance().orElse(Alliance.Blue);
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
