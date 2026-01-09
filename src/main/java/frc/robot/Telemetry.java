package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
  private final double m_maxSpeed;

  /**
   * Construct a telemetry object, with the specified max speed of the robot.
   *
   * @param maxSpeed Maximum speed in meters per second
   */
  public Telemetry(double maxSpeed) {
    m_maxSpeed = maxSpeed;
    SignalLogger.start();

    /* Set up the module state Mechanism2d telemetry */
    for (int i = 0; i < 4; ++i) {
      SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
    }
  }

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();

  /* Robot swerve drive state */
  private final NetworkTable m_driveStateTable = m_inst.getTable("DriveState");
  private final StructPublisher<Pose2d> m_drivePose =
      m_driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
  private final StructPublisher<ChassisSpeeds> m_driveSpeeds =
      m_driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> m_driveModuleStates =
      m_driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> m_driveModuleTargets =
      m_driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModulePosition> m_driveModulePositions =
      m_driveStateTable
          .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
          .publish();
  private final DoublePublisher m_driveTimestamp =
      m_driveStateTable.getDoubleTopic("Timestamp").publish();
  private final DoublePublisher m_driveOdometryFrequency =
      m_driveStateTable.getDoubleTopic("OdometryFrequency").publish();

  /* Robot pose for field positioning */
  private final NetworkTable m_table = m_inst.getTable("Pose");
  private final DoubleArrayPublisher m_fieldPub =
      m_table.getDoubleArrayTopic("robotPose").publish();
  private final StringPublisher m_fieldTypePub = m_table.getStringTopic(".type").publish();

  /* Mechanisms to represent the swerve module states */
  private final Mechanism2d[] m_moduleMechanisms =
      new Mechanism2d[] {
        new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1),
      };
  /* A direction and length changing ligament for speed representation */
  private final MechanismLigament2d[] m_moduleSpeeds =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
      };
  /* A direction changing and length constant ligament for module direction */
  private final MechanismLigament2d[] m_moduleDirections =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      };

  private final double[] m_poseArray = new double[3];

  /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the swerve drive state */
    m_drivePose.set(state.Pose);
    m_driveSpeeds.set(state.Speeds);
    m_driveModuleStates.set(state.ModuleStates);
    m_driveModuleTargets.set(state.ModuleTargets);
    m_driveModulePositions.set(state.ModulePositions);
    m_driveTimestamp.set(state.Timestamp);
    m_driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

    /* Also write to log file */
    SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
    SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
    SignalLogger.writeStructArray(
        "DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
    SignalLogger.writeStructArray(
        "DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
    SignalLogger.writeStructArray(
        "DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
    SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

    /* Telemeterize the pose to a Field2d */
    m_fieldTypePub.set("Field2d");

    m_poseArray[0] = state.Pose.getX();
    m_poseArray[1] = state.Pose.getY();
    m_poseArray[2] = state.Pose.getRotation().getDegrees();
    m_fieldPub.set(m_poseArray);

    /* Telemeterize each module state to a Mechanism2d */
    for (int i = 0; i < 4; ++i) {
      m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * m_maxSpeed));
    }
  }
}
