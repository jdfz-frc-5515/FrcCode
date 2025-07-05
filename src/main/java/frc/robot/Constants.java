package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
// import com.thethriftybot.Conversion;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
// import frc.robot.Library.MUtils.SegmentOnTheField;
import frc.robot.Library.MUtils.SegmentOnTheField;

import java.awt.geom.Point2D;

public class Constants {
    public static String LIME_LIGHT_ARPIL_TAG_NAME_RIGHT = "limelight-right";
    public static String LIME_LIGHT_ARPIL_TAG_NAME_LEFT = "limelight-left";
    public static String LIME_LIGHT_OBJECT_DETECTION = "limelight-up";

    public static class GlobalConstants {
        public final static float INF = (float) Math.pow(10, 6); // this was defined for the 1690 lib
    }

    public static class DriveConstants {
        public final static double kInnerDeadband = 0.004;
        public final static double kOuterDeadband = 0.98; // these were defined for the 1706 lib;
        public static final int IntakeButton = 0;
    }

    public static class SwerveConstants {

        /* IMPORTANT: Tune the motor gains in TunerConstants.java!!! */

        public static final double MaxSpeed = 0.;

        public static final double trackWidth = 0.55245;
        public static final double wheelBase = 0.55245;
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    }

    public static final class PoseEstimatorConstants {
        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.3, 0.3, 0.1);
        public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
        public static final InterpolatingDoubleTreeMap tAtoDev = new InterpolatingDoubleTreeMap();

        public static final Point2D[] tAtoDevPoints = {
                // new Point2D(0.374, 0.003),
                // new Point2D(0.071, 0.2),
                // new Point2D(0.046, 0.4)
                new Point2D.Double(0.17, 0.08),
                new Point2D.Double(0.12, 0.20),
                new Point2D.Double(0.071, 0.35),
                new Point2D.Double(0.046, 0.4),
        };

    }

    public static final class GIntakeConstants {
        public static final int GIntakeTurnID = 16;
        public static final int GIntakeDriveID = 15;
        public static final int GIntakeCCID = 19;
        public static final double retractCCRotation = 0.0;
        public static final double expandCCRotation = 1.4;
        public static final class turnMotorConst {
            public static final double kP = 4;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
        }
    }

    public static final class AutoConstants {
        // Path Following
        public static final double followPathTranslationkP = 5.; // TODO
        public static final double followPathTranslationkI = 0.; // TODO
        public static final double followPathTranslationkD = 0.; // TODO

        public static final double followPathRotationkP = 5.; // TODO
        public static final double followPathRotationkI = 0.; // TODO
        public static final double followPathRotationkD = 0.; // TODO

        public static final PathConstraints generatedPathCommandConstraints = new PathConstraints( // TODO
                3.,
                3.,
                Units.degreesToRadians(540.),
                Units.degreesToRadians(720.));

        // Move To Pose
        public static final double moveToPoseTranslationkP = 5.; // TODO
        public static final double moveToPoseTranslationkI = 0.; // TODO
        public static final double moveToPoseTranslationkD = 0.; // TODO

        public static final double moveToPoseRotationkP = 5.; // TODO
        public static final double moveToPoseRotationkI = 0.; // TODO
        public static final double moveToPoseRotationkD = 0.; // TODO

        public static final double moveToPoseRotationToleranceRadians = Units.degreesToRadians(3.); // TODO
        public static final double moveToPoseTranslationToleranceMeters = 0.02; // TODO

        public static final double maxMoveToSpeed = 1.; // TODO
        public static final double maxMoveToAngularVelocity = Units.degreesToRadians(100.); // TODO

    }

    public static void initializeConstants() {
        for (var p : PoseEstimatorConstants.tAtoDevPoints)
            PoseEstimatorConstants.tAtoDev.put(p.getX(), p.getY());
    }

    public static final class FieldConstants {

        public static final Translation2d FieldCenter = new Translation2d(17.548225 / 2, 8.0518 / 2.);

        // public static final double elevatorHeights[] = { //TODO
        // 0.,
        // 0.3,
        // 0.46,
        // 0.82,
        // 1.39
        // };

        // public static final double elevatorHeightInAdvance = 0.1; //TODO

        // public static final double elevatorHeightsInAdvance[] = {
        // 0.,
        // 0., //0.3,
        // 0.0, //0.42
        // 0.0, //0.82
        // 0.0 //0.65
        // };

        public static final Translation2d BlueReefCenterPos = new Translation2d(4.489323, 8.0518 / 2.); // TODO
        public static final Translation2d DReefTranslation12 = new Translation2d(1.31, 0.161); // TODO
                                                                                               // Translation2d(1.32,
                                                                                               // 0.162)

        public static final double reefTranslationAdjustmentRange = 0.15;
        public static final double reefRotationAdjustmentRangeDegs = 20;

        public static final Translation2d BlueRightStationCenterPos = new Translation2d(1.2, 1.05);
        public static final Translation2d DStationTranslationRSL = new Translation2d(-0.675, 0.42);
        public static final Rotation2d DStationRotationRSL = Rotation2d.fromRadians(0.935);

        public static Pose2d rotateAroundCenter(Pose2d pose, Translation2d centre, Rotation2d rotation) {
            return new Pose2d(pose.getTranslation().rotateAround(centre, rotation), pose.getRotation().plus(rotation));
        }

        public static final double elevatorAlgaeRemovalHeights[] = {
                0.,
                0.,
                0.5 // TODO
        };

        public static final Translation2d BlueRghtStationStPos = new Translation2d(0.6, 1.31);
        public static final Translation2d BlueRghtStationEdPos = new Translation2d(1.6, 0.56);

        public static final Translation2d BlueLeftStationStPos = new Translation2d(BlueRghtStationStPos.getX(),
                FieldCenter.getY() * 2 - BlueRghtStationStPos.getY());
        public static final Translation2d BlueLeftStationEdPos = new Translation2d(BlueRghtStationEdPos.getX(),
                FieldCenter.getY() * 2 - BlueRghtStationEdPos.getY());

        public static final SegmentOnTheField BlueRghtStation = new SegmentOnTheField(BlueRghtStationStPos,
                BlueRghtStationEdPos);
        public static final SegmentOnTheField BlueLeftStation = new SegmentOnTheField(BlueLeftStationStPos,
                BlueLeftStationEdPos);

        public static final double StationDetectionArea = 0.3;

    }

    public static final class TurningArm {
        public static final int motorID = 13;
        public static final int canCoderID = 14;
        public static final String canBusName = "rio";
        public static final double KP = 7;
        public static final double KI = 1.0;
        public static final double KD = 0.1;
        public static final double KV = 0.0; // 0.1
        public static final double KS = 0.0;
        public static final double KA = 0.0;
        public static final double Velocity = 10; // 100.0;
        public static final double Acceleration = 20; // 600.0;
        public static final double Jerk = 0.0;
        public static final double SensorToMechanismRatio = 1.0;
        public static final double RotorToSensorRatio = 44;
        // bigger value means arm is more expanded(clockwise)
        public static final double zeroPos = 0;
        public static final double basePos = 0.4146980;
        public static final double l1Pos = 1.039307;//0.407227;
        public static final double l2Pos = 1.039307;
        public static final double dodgePos =  1.139307;  // the pos that allows elevator to move
        public static final double l3Pos = 1.039307;
        public static final double l4Pos = 0.6;
        public static final double ball1Pos = 4.3;
        public static final double ball2Pos = 4.3;
    }

    public static final class Elevator {
        public static final int primaryMotorID = 15;
        public static final int followerMotorID = 16;
        public static final int canCoderID = 17;
        public static final String canBusName = "rio";
        public static class Up {
            public static final double KP = 5; // 25;  // 5; // 20.0;
            public static final double KI = 0; //3.2; // 0;
            public static final double KD = 0.1; //0.2;
            public static final double KV = 0.5;//1;//0.1;
            public static final double KS = 0.05;//0.05;
            public static final double KA = 0;//0.07;
        }
        public static class Down {
            public static final double KP = 5; // 10.0;
            public static final double KI = 0;
            public static final double KD = 0.1;
            public static final double KV = 1;//1;//0.1;
            public static final double KS = 0;//0.05;
            public static final double KA = 0;//0.07;
        }

        public static final double Velocity = 30; // 30;//50;
        public static final double Acceleration = 30;// 100;// 75;
        public static final double Jerk = 0;
        public static final double SensorToMechanismRatio = 1.0;
        public static final double RotorToSensorRatio = 4.8;//11.33;
        // smaller value means higher position
        public static final double zeroPos = 0;
        public static final double basePos = 0;

        public static final double l1Pos = -4.41582; // -1.75;
        public static final double l2Pos = -4.41582;
        public static final double l3Pos = -8.385644;
        public static final double l4Pos = -14;
        public static final double ball1Pos = -2.8;
        public static final double ball2Pos = -7;
        public static final double upDodgePos = -6.5;
        public static final double downDodgePos = -3.47;
    }


    public static final class Intake {
        public static final int motorID = 18;
        public static final String canBusName = "rio";
        public static final double KP = 0.5;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double coralInSpeed = 15;
        public static final double coralInSlowSpeed = 2;
        public static final double coralInReverseSpeed = -3;
        public static final double coralOutSpeed = 30;
        public static final double BallInSpeed = -40;
        public static final double BAllOutSpeed = 15;
    }

    public static final class Candle {
        public static final int candleID = 19;
        public static final String canBusName = "rio";
    }

    public static final class PathPlanner {
        public static final double constraintsSpeed = 2.;
        public static final double constraintsAccel = 2.;
    }
}
