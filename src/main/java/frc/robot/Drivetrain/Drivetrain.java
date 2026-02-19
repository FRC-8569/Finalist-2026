package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import dev.doglog.DogLog;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.GlobalConstant;
import frc.robot.Drivetrain.Constants.Modules;
import frc.robot.Vision.Vision;
import frc.utils.FieldObjects;
import frc.utils.PoseUtils;
import frc.utils.PoseUtils.FieldPlace;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem{
    public static Drivetrain inst;
    public Time SimLoopTime = Milliseconds.of(1);
    private boolean hasDirectionUpdated = false;
    private Notifier SimNotifier;
    private SwerveRequest.ApplyRobotSpeeds autoDrive;
    public SwerveRequest.FieldCentricFacingAngle manualFacing;
    public SwerveRequest.FieldCentric manualDrive;
    public SimulatedDrive SimulatedInstance;
    private Optional<FieldObjects> objects = Optional.empty();
    public Vision vision;

    private double SimTime = 0;

    private Drivetrain(SwerveDrivetrainConstants constants, SwerveModuleConstants<?,?,?>... modules){
        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, Utils.isSimulation() ? SimulatedDrive.regulateModuleConstantsForSimulation(modules) : modules);
        if(Utils.isSimulation()) simulationInit();

        autoDrive = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.Position)
            .withDesaturateWheelSpeeds(true);

        manualDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(Utils.isSimulation() ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withDeadband(Constants.MaxVelocity.times(Constants.Deadband))
            .withRotationalDeadband(RotationsPerSecond.of(0.1))
            .withDesaturateWheelSpeeds(true);

        manualFacing = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(Utils.isSimulation() ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withDeadband(Constants.MaxVelocity.times(Constants.Deadband))
            .withRotationalDeadband(Constants.MaxOmega.times(Constants.Deadband))
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withHeadingPID(Constants.RotationPIDConfig[0], Constants.RotationPIDConfig[1], Constants.RotationPIDConfig[2]);

        vision = Vision.getInstance();
        
        autoInit();
    }

    public Command withHeading(Optional<FieldObjects> obj){
        return runOnce(() -> objects=obj);
    }//

    public Command drive(Supplier<LinearVelocity> vx, Supplier<LinearVelocity> vy, Supplier<AngularVelocity> omega){
        return setControl(() -> objects.isPresent() && omega.get().lt(Constants.MaxOmega.times(Constants.Deadband))? 
        manualFacing.withVelocityX(vx.get()).withVelocityY(vy.get()).withTargetDirection(objects.get().getPose().getRotation().toRotation2d()): 
        manualDrive.withVelocityX(vx.get()).withVelocityY(vy.get()).withRotationalRate(omega.get())).withName("Drive normally");
    }

    public Command drive(Pose2d pose, LinearVelocity endvelocity){
        try{
            return new PathfindingCommand(
            PoseUtils.getPose(pose, DriverStation.getAlliance().orElseThrow()), 
            Constants.AutoConstraints, 
            endvelocity,
            () -> getState().Pose, 
            () -> getState().Speeds,
            (speeds, ff) -> setControl(autoDrive.withSpeeds(speeds).withWheelForceFeedforwardsX(ff.robotRelativeForcesX()).withWheelForceFeedforwardsY(ff.robotRelativeForcesY())), 
            Constants.AutoPID, 
            RobotConfig.fromGUISettings(), 
            this);
        }catch(Exception e){
            DriverStation.reportWarning(e.getMessage(), e.getStackTrace());
            return idle();
        }
    }

    public Command driveOverTrench(){
        return defer(() -> {
            return drive(getState().Pose.nearest(List.of(PoseUtils.getPose(GlobalConstant.LeftTrench, DriverStation.getAlliance().orElseThrow()),PoseUtils.getPose(GlobalConstant.RightTrench, DriverStation.getAlliance().orElseThrow()))),MetersPerSecond.of(1));
        }); 
    }

    public Command driveToTower(){
        return defer(() -> {
            List<Pose2d> towerpose = List.of(PoseUtils.getPose(new Pose2d(15,3.9,Rotation2d.kZero), DriverStation.getAlliance().orElseThrow()), PoseUtils.getPose(new Pose2d(15,4.7,Rotation2d.kZero), DriverStation.getAlliance().orElseThrow()));
            return drive(getState().Pose.nearest(towerpose), MetersPerSecond.of(0));
        });
    }

    public Command drive(LinearVelocity endVelocity, Pose2d... pose){
         try{
            return new PathfindThenFollowPath(
                new PathPlannerPath(
                    PathPlannerPath.waypointsFromPoses(Arrays.asList(pose).stream().map(p -> PoseUtils.getPose(p, DriverStation.getAlliance().orElseThrow())).toList()),
                    Constants.AutoConstraints, 
                    null, 
                    new GoalEndState(endVelocity, getState().Pose.getRotation())),
                Constants.AutoConstraints, 
                () -> getState().Pose, 
                () -> getState().Speeds, 
                (speeds, ff) -> setControl(autoDrive.withSpeeds(speeds).withWheelForceFeedforwardsX(ff.robotRelativeForcesX()).withWheelForceFeedforwardsY(ff.robotRelativeForcesY())), 
                Constants.AutoPID, 
                RobotConfig.fromGUISettings(), 
                () -> false,
                this);
        }catch(Exception e){
            DriverStation.reportWarning(e.getMessage(), e.getStackTrace());
            return idle();
        }
    }

    public Command setControl(Supplier<SwerveRequest> req){
        return run(() -> {
            setControl(req.get());
            DogLog.log("Drivetrain/ControlMode", req.get().getClass().getSimpleName());
            
        });
    }

    @Override
    public Command idle(){
        return setControl(() -> new SwerveRequest.Idle());
    }

    @Override
    public void periodic(){
        if(DriverStation.isDisabled() || !hasDirectionUpdated){
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? GlobalConstant.RedAlliance
                        : GlobalConstant.BlueAlliance
                );
                hasDirectionUpdated = true;
            });
        }

        DogLog.log("Drivetrain/ModuleState", getState().ModuleStates);
        DogLog.log("Drivetrain/ModuleTarget", getState().ModuleTargets);
        DogLog.log("Drivetrain/RobotPose", getState().Pose);
        DogLog.log("Drivetrain/RobotSpeeds", getState().Speeds);
        DogLog.log("Drivetrain/LockingTarget", objects.isEmpty() ? "null" : objects.get().toString());
        DogLog.log("Drivetrain/LockObjectPose", objects != null && objects.isPresent() ? PoseUtils.getPose(GlobalConstant.HubPose.toPose2d(), DriverStation.getAlliance().orElseThrow()) : null);
        DogLog.log("Drivetrain/LockingAngle", manualFacing.TargetDirection);
        DogLog.log("Drivetrain/LockingError", manualFacing.TargetDirection.minus(getState().Pose.getRotation()));
        PoseUtils.getRobotZone().ifPresent(z -> DogLog.log("Drivetrain/CurrentPlace", z.toString()));
        
        IntStream.range(0, getModules().length).forEachOrdered((i) -> {
            DogLog.log("Drivetrain/Module/%d/SteerMotorVelocity".formatted(i), getModule(i).getSteerMotor().getVelocity().getValue());
            DogLog.log("Drivetrain/Module/%d/SteerMotorAcceleration".formatted(i), getModule(i).getSteerMotor().getAcceleration().getValue());
            DogLog.log("Drivetrain/Module/%d/SteerError".formatted(i), getModule(i).getTargetState().angle.minus(getModule(i).getCurrentState().angle).getDegrees(),Degrees);
            DogLog.log("Drivetrain/Module/%d/SteerAngle".formatted(i), getModule(i).getEncoder().getAbsolutePosition().getValue().in(Degrees), Degree);
        });
        
        vision.getPose().ifPresent(s -> addVisionMeasurement(s.getFirst(), s.getSecond()));
        try{
            DogLog.log("Drivetrain/CurrentDoing", getCurrentCommand().getName());
        }catch(NullPointerException e){
            DogLog.log("Drivetrain/CurrentDoing", "nothing");
        }

    }

    public Command resetHeading(){
        return runOnce(() -> {
            resetRotation(DriverStation.getAlliance().orElseThrow() == Alliance.Red ? GlobalConstant.RedAlliance : GlobalConstant.BlueAlliance);
            setOperatorPerspectiveForward(DriverStation.getAlliance().orElseThrow() == Alliance.Red ? GlobalConstant.RedAlliance : GlobalConstant.BlueAlliance);
        });
    }

    public Command fieldReset(){
        return runOnce(() -> {
            resetPose(DriverStation.getAlliance().orElseThrow() == Alliance.Blue ? 
            new Pose2d(Constants.RobotSize[0].div(2),Constants.RobotSize[1].div(2), GlobalConstant.RedAlliance.rotateBy(Rotation2d.k180deg)) : 
            new Pose2d(Meters.of(16.566802).minus(Constants.RobotSize[0].div(2)), Meters.of(8.072088).minus(Constants.RobotSize[1].div(2)), GlobalConstant.BlueAlliance.rotateBy(Rotation2d.k180deg)));
        });
    }

    @Override
    public void resetPose(Pose2d Pose){
        if(SimulatedInstance != null) SimulatedInstance.mapleSimDrive.setSimulationWorldPose(Pose);
        super.resetPose(Pose);
    }

    private void simulationInit(){
        SimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        SimNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - SimTime;
            SimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        SimNotifier.startPeriodic(SimLoopTime.in(Seconds));
    }

    private void autoInit(){
        try{
            AutoBuilder.configure(
                () -> getState().Pose, 
                this::resetPose, 
                () -> getState().Speeds, 
                (speeds, ff) -> autoDrive.withSpeeds(speeds).withWheelForceFeedforwardsX(ff.robotRelativeForcesX()).withWheelForceFeedforwardsY(ff.robotRelativeForcesY()), 
                Constants.AutoPID, 
                RobotConfig.fromGUISettings(), 
                () -> DriverStation.getAlliance().orElseThrow() == Alliance.Blue, 
                this);
        }catch(Exception e){
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
    }

    public static Drivetrain getInstance(){
        inst = inst == null ? new Drivetrain(Constants.constants,Modules.FrontLeft.constant, Modules.FrontRight.constant, Modules.BackLeft.constant, Modules.BackRight.constant) : inst;
        return inst;
    }

    public boolean inZone(){
        return (DriverStation.getAlliance().orElseThrow() == Alliance.Red && PoseUtils.getRobotZone().get() == FieldPlace.RedZone) || DriverStation.getAlliance().orElseThrow() == Alliance.Blue && PoseUtils.getRobotZone().get() == FieldPlace.BlueZone;
    }
}
