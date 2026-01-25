package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static class SwerveConstants {
    public static final double kDrivetrainWidth = Units.inchesToMeters(26.94);
    public static final double kDrivetrainLength = Units.inchesToMeters(26.94);
    public static final double kRobotWidth = Units.inchesToMeters(33.876000);
    public static final double kRobotLength = Units.inchesToMeters(33.876000);
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_LEFT_ID = 15;
    public static final int SHOOTER_MOTOR_RIGHT_ID = 16;
    public static final int SHOOTER_MOTOR_TOP_ROLLER_ID = 17;

    public static final int SHOOTER_SUPPLY_LIMIT = 60;
    public static final int SHOOTER_STATOR_LIMIT = 80;
  }

  public static class VisionConstants {
    public static final String kFrontCameraName = "FrontCamera";
    public static final String kSideCameraName = "SideCamera";
    public static final Transform3d kFrontRobotToCam =
        new Transform3d(
            new Translation3d(-0.342138, 0.342138, 0.7112),
            new Rotation3d(0, Units.degreesToRadians(-15), 0));
    public static final Transform3d kSideRobotToCam =
        new Transform3d(
            new Translation3d(-0.342138, 0.342138, 0.7112),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-90)));
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }
}
