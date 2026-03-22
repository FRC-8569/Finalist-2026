package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import dev.doglog.DogLog;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.GlobalConstants;
import frc.robot.Drivetrain.Constants.Modules;
import frc.robot.Shooter.Shooter;
import frc.robot.Vision.Vision;
import frc.utils.Tools;
import frc.utils.Tools.FieldPlace;
import frc.utils.Tools.FieldSide;

public class Drivetrain extends SwerveDrivetrain<TalonFX,TalonFX, CANcoder> implements Subsystem{
    private static Drivetrain inst;
    private Time SimLoopTime = Milliseconds.of(1);
    private boolean hasDirectionUpdated = false;
    private Notifier SimNotifier;
    public SwerveRequest.ApplyRobotSpeeds AutoDrive;
    public SwerveRequest.FieldCentric ManualDrive;
    public SwerveRequest.FieldCentricFacingAngle ManualFacing;
    private SwerveRequest.RobotCentric ManualRobotCentric;
    private double SimTime = 0;
    private boolean faceLock = false;
    private boolean isRobotCentric = false;

    public Vision vision;
    
    private Drivetrain(){
        
        super(TalonFX::new,TalonFX::new,CANcoder::new,Constants.constants,
       
             new SwerveModuleConstants<?, ?, ?>[] {
                Modules.FrontLeft.constant,
                Modules.FrontRight.constant,
                Modules.BackLeft.constant,
                Modules.BackRight.constant
            }
        );


        
        AutoDrive = new SwerveRequest.ApplyRobotSpeeds()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(SteerRequestType.Position)
                .withDesaturateWheelSpeeds(true);

        ManualDrive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.Position)
                .withDeadband(Constants.MaxVelocity.times(Constants.Deadband))
                .withRotationalDeadband(Constants.MaxOmega.times(Constants.Deadband))
                .withDesaturateWheelSpeeds(true);

        ManualFacing = new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.Position)
                .withDeadband(Constants.MaxVelocity.times(Constants.Deadband))
                .withRotationalDeadband(Constants.MaxOmega.times(Constants.Deadband))
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                .withHeadingPID(Constants.RotationPIDConfig[0], Constants.RotationPIDConfig[1],
                        Constants.RotationPIDConfig[2]);

        ManualRobotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withRotationalDeadband(Constants.MaxOmega.times(Constants.Deadband))
            .withDesaturateWheelSpeeds(true);
        
        autoInit();
        if(Utils.isSimulation()) simInit();
        register();
        vision = Vision.getInstance();
    }

    public Command drive(Supplier<LinearVelocity> vx, Supplier<LinearVelocity> vy, Supplier<AngularVelocity> omega){
        return run(() -> {
            if ( omega.get().abs(RotationsPerSecond) < Constants.MaxOmega.times(Constants.Deadband).abs(RotationsPerSecond)  && inZone() && faceLock)
                setControl(
                    ManualFacing.withVelocityX(vx.get()).withVelocityY(vy.get())
                                .withTargetDirection(Tools.getPredictState().drivetrainFacing())
                );
            else if(omega.get().lt(Constants.MaxOmega.times(Constants.Deadband)) && !inZone() && faceLock)
                setControl(
                ManualFacing.withVelocityX(vx.get()).withVelocityY(vy.get())
                            .withTargetDirection(Rotation2d.k180deg)
                );
            else setControl(
                isRobotCentric ? ManualRobotCentric.withVelocityX(vx.get()).withVelocityY(vy.get()).withRotationalRate(omega.get())
                               : ManualDrive.withVelocityX(vx.get()).withVelocityY(vy.get()).withRotationalRate(omega.get())
            );
        });
        
    }

    public Command drive(Pose2d pose, FieldSide side){
        return defer(() -> {
            try {
                RobotConfig r = RobotConfig.fromGUISettings();
                return new PathfindingCommand(
                        Tools.getPose(pose, side),
                        Constants.AutoConstraints,
                        () -> getState().Pose,
                        () -> getState().Speeds,
                        (speeds, ff) -> setControl(
                                AutoDrive.withSpeeds(speeds).withWheelForceFeedforwardsX(ff.robotRelativeForcesX())
                                        .withWheelForceFeedforwardsY(ff.robotRelativeForcesY())),
                        Constants.AutoPID,
                        r,
                        this).withName("Driving to %.2f %.2f %.2f.".formatted(pose.getX(),pose.getY(),pose.getRotation().getDegrees()));
            } catch (Exception e) {
                DriverStation.reportWarning(e.getMessage(), e.getStackTrace());
                return idle().withName("Driving to %.2f %.2f %.2f but with some error, now idling".formatted(pose.getX(),pose.getY(),pose.getRotation().getDegrees()));
            }
        });
    }

    public Command drive(String name){
        try{
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(name));
            
        }catch(Exception e){
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return idle();
        }
    }
    
    public Command resetHeading(){
        return Commands.runOnce(() -> resetRotation(DriverStation.getAlliance().orElseThrow() == Alliance.Red ? GlobalConstants.RedAlliance : GlobalConstants.BlueAlliance));
    }

    public Command fieldReset(){
        return runOnce(() -> {
            resetPose(DriverStation.getAlliance().orElseThrow() == Alliance.Blue
                    ? new Pose2d(Constants.RobotSize[0].div(2), Constants.RobotSize[1].div(2),
                            GlobalConstants.RedAlliance.rotateBy(Rotation2d.k180deg))
                    : new Pose2d(Meters.of(16.566802).minus(Constants.RobotSize[0].div(2)),
                            Meters.of(8.072088).minus(Constants.RobotSize[1].div(2)),
                            GlobalConstants.BlueAlliance.rotateBy(Rotation2d.k180deg)));
        }).ignoringDisable(true);
    }

    @Override
    public void periodic(){
        if (DriverStation.isDisabled() || !hasDirectionUpdated) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? GlobalConstants.RedAlliance
                                : GlobalConstants.BlueAlliance);
                hasDirectionUpdated = true;
            });
        }

        if((getState().Pose.getRotation().getMeasure().isNear(Degrees.of(0), Degrees.of(10)) || getState().Pose.getRotation().getMeasure().isNear(Degrees.of(180), Degrees.of(10))) && getState().Pose.getMeasureY().isNear(Meters.of(4), Meters.of(1))) updateVisionPose().schedule();

        vision = Vision.getInstance();
        DogLog.log("Driver/Vision/IsVisionAvalible", vision == null);

        if(vision != null){
            vision.getRobotPose().ifPresent(s -> addVisionMeasurement(s.getFirst(), s.getSecond()));
        }

        log();
    }

    public void log(){
        DogLog.log("Driver/Drivetrain/RobotPose", getState().Pose);
        DogLog.log("Driver/Drivetrain/ModuleState", getState().ModuleStates);

        DogLog.log("Debug/Drivetrain/ModuleTarget", getState().ModuleTargets);
        DogLog.log("Debug/Drivetrain/LockingError", ManualFacing.TargetDirection.minus(getState().Pose.getRotation()));
        DogLog.log("Debug/Drivetrain/TimeStamp", getState().Timestamp);
        DogLog.log("Driver/Drivetrain/isFaceLock", faceLock);
        DogLog.log("Debug/Drivetrain/ChassisForce", Arrays.stream(getModules()).map(m -> m.getDriveMotor().getStatorCurrent().getValueAsDouble()*m.getDriveMotor().getMotorKT().getValueAsDouble()/Constants.WheelRadius.in(Meters)/GlobalConstants.G.in(MetersPerSecondPerSecond)).mapToDouble(Double::doubleValue).sum());;
        DogLog.log("Driver/Drivetrain/RobotSide", getSide().toString());
    }

    public Command updateVisionPose(){
        return Commands.runOnce(() -> {
            if(vision != null){
                Optional<Pair<Pose2d,Double>> res = vision.getRobotPose();
                res.ifPresent(r -> resetPose(r.getFirst()));
                DogLog.log("Driver/Vision/isUseVisionPose", true);
                resetHeading();
            }else{
                DogLog.log("Driver/Vision/isUseVisionPose", false);
            }
        }).ignoringDisable(true);
    }

    public Command faceLock(){
        return Commands.runEnd(
            () -> this.faceLock = true, 
            () -> this.faceLock = false).onlyIf(() -> DriverStation.isEnabled());
    }
    public Command robotCentric(){
        return runEnd(
            () -> isRobotCentric = true, 
            () -> isRobotCentric = false);
    }

    private void autoInit(){
        try{
            AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose,
                () -> getState().Speeds,
                (speeds, ff) -> setControl(AutoDrive.withSpeeds(speeds).withWheelForceFeedforwardsX(ff.robotRelativeForcesX()).withWheelForceFeedforwardsY(ff.robotRelativeForcesY())),
                Constants.AutoPID,
                RobotConfig.fromGUISettings(),
                () -> DriverStation.getAlliance().orElseThrow() == Alliance.Blue,
                this
            );
        }catch(Exception e){
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
    }

    /**
     * initialize the simulation
     */
    private void simInit(){
        SimTime = Utils.getCurrentTimeSeconds();
        SimNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - SimTime;
            SimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        SimNotifier.setName("DrivetrainSimNotifier");
        SimNotifier.startPeriodic(SimLoopTime.in(Seconds));
    }

    @Override
    public void setControl(SwerveRequest req){
        super.setControl(req);
        DogLog.log("Driver/Drivetrain/CurrentMode", req.getClass().getSimpleName());
    }

    public static Drivetrain getInstance(){
        inst = inst == null ? new Drivetrain() : inst;
        return inst;
    }

    public double getCurrent(){
        return Arrays.stream(getModules()).map(s -> s.getDriveMotor().getSupplyCurrent().getValueAsDouble()+s.getSteerMotor().getSupplyCurrent().getValueAsDouble()).mapToDouble(Double::doubleValue).sum();
    }
 
    public boolean inZone() {
        return switch(DriverStation.getAlliance().orElse(Alliance.Blue)){
            case Red -> FieldPlace.RedZone;
            case Blue -> FieldPlace.BlueZone;
        } == getPlace();
    }

    public FieldPlace getPlace(){
        if(FieldPlace.RedZone.contains(getState().Pose)) return FieldPlace.RedZone;
        else if(FieldPlace.Neutral.contains(getState().Pose)) return FieldPlace.Neutral;
        else return FieldPlace.BlueZone;
    }

    public FieldSide getSide(){
        Distance dy = GlobalConstants.CenterLine.getSecond().minus(getState().Pose.getMeasureY());
        return
            (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ?
            (dy.gt(Meters.zero()) ? FieldSide.RIGHT : FieldSide.LEFT) :
            (dy.gt(Meters.zero()) ? FieldSide.RIGHT : FieldSide.LEFT)
        ;
    }
}
