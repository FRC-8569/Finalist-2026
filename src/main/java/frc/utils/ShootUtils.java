package frc.utils;

import java.util.Optional;

import dev.doglog.DogLog;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.GlobalConstants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Shooter.Constants;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.Constants.Pitch;
import frc.robot.Shooter.Constants.Shoot;

public class ShootUtils {
    public static Drivetrain drivetrain = Drivetrain.getInstance();
    public static Shooter shoooter = Shooter.getInstance();
    private static Timer timer = new Timer();

    public enum ApproachStrategy {
        /**
         * Calculates the Minimum Velocity required to hit the target.
         * This results in a High-Arc (Lob) trajectory.
         * The velocity parameter is ignored for this strategy.
         */
        MIN_VELOCITY,

        /**
         * Calculates the Low-Arc (Direct) trajectory for a specific velocity.
         * Uses the lower of the two possible pitch angles.
         */
        LOW_ARC
    }


    public static record RobotState(Rotation2d facing, LinearVelocity vel, Angle pitch){
        public boolean isNear(RobotState other){
            return Math.abs(other.facing.getDegrees()-this.facing.getDegrees()) < 10 && other.vel.isNear(vel, 0.05) && other.pitch.isNear(pitch, 0.05);
        }
    };

    /**
     * Legacy convenience method. Defaults to MIN_VELOCITY strategy.
     * @param Target the Target transform
     * @return Optional RobotState
     */
    public static Optional<RobotState> calculateState(Transform3d Target){
        return calculateRobotState(Target, ApproachStrategy.MIN_VELOCITY, 0.0);
    }

    public static Optional<RobotState> calculateState(FieldObjects obj){
        return calculateRobotState(new Transform3d(new Pose3d(drivetrain.getState().Pose).plus(Constants.ShooterPlace), obj.getPose()), ApproachStrategy.MIN_VELOCITY, 0);
    }
    
    /**
     * Calculates the shooter and drivetrain state based on the selected strategy.
     * 
     * @param Target The target transform in {@link #Transform3d}
     * @param strategy The shooting strategy to use (MIN_VELOCITY or LOW_ARC)
     * @param velocity The desired launch velocity (Meters/Second). Required for LOW_ARC, ignored for MIN_VELOCITY.
     * @return Optional containing the state, or empty if no valid solution exists.
     */
    public static Optional<RobotState> calculateRobotState(Transform3d Target, ApproachStrategy strategy, double velocity) {
        switch (strategy) {
            case LOW_ARC:
                return solveLowArc(Target, velocity);
            case MIN_VELOCITY:
            default:
                return solveMinVelocity(Target);
        }
    }

    /**
     * Solves for the Minimum Velocity trajectory using Newton's Method.
     */
    private static Optional<RobotState> solveMinVelocity(Transform3d Target) {
        Angle yaw = Target.getRotation().getMeasureZ();
        double distance = Math.hypot(Target.getX(), Target.getY());
        double z = Target.getZ();

        final int maxIters = 100;
        final double tol = Degrees.of(0.1).in(Radians);   // radians
        final double eps = 1e-9;

        if (!Double.isFinite(distance) || distance < eps || !Double.isFinite(z)) {
            return Optional.empty();
        }

        // Initial guess (radians)
        // Use 45 degrees (PI/4) as a robust start to avoid singularity at z=0
        double pPrev = Math.PI / 4.0;
        double p = pPrev;
        boolean converged = false;

        // Solve A(theta) = distance*cos(2theta) + z*sin(2theta) = 0
        // (stationary point of required launch speed)
        timer.reset();
        timer.start();
        for (int i = 0; i < maxIters; i++) {
            double s2 = Math.sin(2.0 * pPrev);
            double c2 = Math.cos(2.0 * pPrev);

            double A = distance * c2 + z * s2;
            double dA = -2.0 * distance * s2 + 2.0 * z * c2;

            if (!Double.isFinite(A) || !Double.isFinite(dA) || Math.abs(dA) < eps) {
                return Optional.empty();
            }

            double pNext = pPrev - (A / dA); // standard Newton step
            if (!Double.isFinite(pNext)) {
                return Optional.empty();
            }

            p = pNext;
            if (Math.abs(pNext - pPrev) < tol) {
                converged = true;
                break;
            }
            pPrev = pNext;
            DogLog.log("BoringMath/ShooterCalculation/NewtonMethod", pPrev);
            DogLog.log("BoringMath/ShooterCalculation/CalculateTime", timer.get());
        }
        timer.stop();

        if (!converged) {
            return Optional.empty();
        }

        double v = getVelocity(Target, Radians.of(p));

        if(Math.max(Pitch.PitchingAngle.getFirst().in(Radians), Math.min(p, Pitch.PitchingAngle.getSecond().in(Radians))) != p || MetersPerSecond.of(v).gt(Shoot.MaxVelocity)) return Optional.empty();
        
        return Optional.of(new RobotState(new Rotation2d(yaw), MetersPerSecond.of(v), Radians.of(p)));
    }

    /**
     * Solves for the Low-Arc (Direct) trajectory using the quadratic formula.
     */
    private static Optional<RobotState> solveLowArc(Transform3d Target, double launchVelocity) {
        Angle yaw = Target.getRotation().getMeasureZ();
        double distance = Math.hypot(Target.getX(), Target.getY());
        double z = Target.getZ();
        
        double g = GlobalConstants.G.abs(MetersPerSecondPerSecond);

        double v = launchVelocity;
        double v2 = v * v;

        // Coefficients for: a*tan^2(theta) + b*tan(theta) + c = 0
        // Derived from projectile motion equation solving for angle given velocity:
        // z = x*tan(theta) - (g*x^2)/(2*v^2)*(1 + tan^2(theta))
        double term = (g * distance * distance) / (2.0 * v2);
        double a = term;
        double b = -distance;
        double c = z + term;

        double discriminant = b * b - 4.0 * a * c;

        if (discriminant < 0.0) {
            // Target is out of range for the given velocity
            return Optional.empty();
        }

        // We want the smaller angle (Low Arc), which corresponds to the smaller root
        // tan(theta) = (-b - sqrt(delta)) / (2a)
        double tanTheta = (-b - Math.sqrt(discriminant)) / (2.0 * a);
        double theta = Math.atan(tanTheta);


        if(Math.max(Pitch.PitchingAngle.getFirst().in(Radians), Math.min(theta, Pitch.PitchingAngle.getSecond().in(Radians))) != theta || MetersPerSecond.of(v).gt(Shoot.MaxVelocity)) return Optional.empty();
        

        return Optional.of(new RobotState(
            new Rotation2d(yaw), 
            MetersPerSecond.of(v), 
            Radians.of(theta)
        ));
    }

    private static double getVelocity(Transform3d Target, Angle PitchAngle){
        Rotation2d angle = new Rotation2d(PitchAngle);
        double distance = Math.hypot(Target.getX(), Target.getY());
        double z = Target.getZ();

        double denom =
            distance * angle.times(2).getSin()
            - 2.0 * z * Math.pow(angle.getCos(), 2);

        if (!Double.isFinite(denom) || denom <= 0.0) {
            return Double.NaN;
        }

        double g = GlobalConstants.G.abs(MetersPerSecondPerSecond);
        return Math.sqrt(g * distance * distance / denom);
    }
}