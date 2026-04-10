// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FuelConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ShootingOnTheFly;
import frc.robot.util.ShootingOnTheFly.SOTFResult;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootOnTheMove extends Command {
  private final DoubleSupplier m_forwardSupplier;
  private final DoubleSupplier m_strafeSupplier;

  private final Swerve m_swerve;
  private final Shooter m_shooter;

  private final SwerveRequest.FieldCentricFacingAngle m_swerveRequest =
      new FieldCentricFacingAngle()
          .withHeadingPID(
              SwerveConstants.kRotationP, SwerveConstants.kRotationI, SwerveConstants.kRotationD);

  private static final StructPublisher<Pose2d> m_sotfVirtualTargetPub;
  private static final StructPublisher<Pose2d> m_sotfGoalPub;
  private static final DoublePublisher m_sotfStaticDistancePub;
  private static final DoublePublisher m_sotfVirtualDistancePub;
  private static final DoublePublisher m_sotfTimeOfFlightPub;
  private static final IntegerPublisher m_sotfIterationsPub;
  private static final BooleanPublisher m_sotfConvergedPub;
  private static final DoublePublisher m_sotfRobotSpeedPub;
  private static final DoublePublisher m_sotfAimingAngleDegPub;

  static {
    NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("Shooter");
    NetworkTable sotfTable = shooterTable.getSubTable("SOTF");
    m_sotfVirtualTargetPub = sotfTable.getStructTopic("VirtualTarget", Pose2d.struct).publish();
    m_sotfGoalPub = sotfTable.getStructTopic("Goal", Pose2d.struct).publish();
    m_sotfStaticDistancePub = sotfTable.getDoubleTopic("StaticDistanceM").publish();
    m_sotfVirtualDistancePub = sotfTable.getDoubleTopic("VirtualDistanceM").publish();
    m_sotfTimeOfFlightPub = sotfTable.getDoubleTopic("TimeOfFlightS").publish();
    m_sotfIterationsPub = sotfTable.getIntegerTopic("Iterations").publish();
    m_sotfConvergedPub = sotfTable.getBooleanTopic("Converged").publish();
    m_sotfRobotSpeedPub = sotfTable.getDoubleTopic("RobotSpeedMps").publish();
    m_sotfAimingAngleDegPub = sotfTable.getDoubleTopic("AimingAngleDeg").publish();
  }

  /** Creates a new ShootOnTheMove. */
  public ShootOnTheMove(
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      double deadband,
      Swerve swerve,
      Shooter shooter) {
    m_forwardSupplier = forwardSupplier;
    m_strafeSupplier = strafeSupplier;
    m_swerve = swerve;
    m_shooter = shooter;

    m_swerveRequest.Deadband = deadband;

    addRequirements(swerve, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveDriveState driveState = m_swerve.getState();
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(driveState.Speeds, driveState.Pose.getRotation());

    Translation2d goalPosition = Constants.FieldSpots.getHubPosition();

    SOTFResult result =
        ShootingOnTheFly.calculate(
            driveState.Pose,
            fieldSpeeds,
            m_swerve.getAcceleration(),
            goalPosition,
            ShooterConstants.kSOTFLatencyCompensation,
            ShooterConstants.TIME_OF_FLIGHT_MAP);

    Translation2d virtualTarget = result.virtualTarget();
    m_sotfVirtualTargetPub.set(new Pose2d(virtualTarget, result.aimingAngle()));
    m_sotfGoalPub.set(new Pose2d(goalPosition, new Rotation2d()));
    double staticDistance =
        goalPosition.getDistance(driveState.Pose.getTranslation())
            - FuelConstants.kLookupTableDistanceOffset
            - FuelConstants.getAllianceLookupOffset();
    m_sotfStaticDistancePub.set(staticDistance);
    m_sotfVirtualDistancePub.set(result.virtualDistance());
    m_sotfTimeOfFlightPub.set(result.timeOfFlight());
    m_sotfIterationsPub.set(result.iterations());
    m_sotfConvergedPub.set(result.converged());
    double robotSpeed =
        Math.hypot(driveState.Speeds.vxMetersPerSecond, driveState.Speeds.vyMetersPerSecond);
    m_sotfRobotSpeedPub.set(robotSpeed);
    m_sotfAimingAngleDegPub.set(result.aimingAngle().getDegrees());

    // Convert virtual target distance to lookup table reference frame
    // (front-of-bumper to front-of-hub)
    double lookupDistance =
        Math.max(
            0,
            result.virtualTarget().getDistance(driveState.Pose.getTranslation())
                - FuelConstants.kLookupTableDistanceOffset
                - FuelConstants.getAllianceLookupOffset());

    double bottomSpeedRps = ShooterConstants.BOTTOM_SHOOTER_SPEED_MAP.get(lookupDistance) / 60.0;
    double topRollerSpeedRps = ShooterConstants.TOP_ROLLER_SPEED_MAP.get(lookupDistance) / 60.0;

    m_shooter.setTargetSpeeds(bottomSpeedRps, topRollerSpeedRps);

    double forwardSpeed = m_forwardSupplier.getAsDouble();
    double strafeSpeed = m_strafeSupplier.getAsDouble();

    Rotation2d targetAngle = result.aimingAngle();
    if (!Constants.ifOnBlue()) {
      targetAngle = targetAngle.rotateBy(Rotation2d.k180deg);
    }

    m_swerve.setControl(
        m_swerveRequest
            .withVelocityX(forwardSpeed)
            .withVelocityY(strafeSpeed)
            .withTargetDirection(targetAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.setControl(m_swerveRequest.withVelocityX(0).withVelocityY(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
