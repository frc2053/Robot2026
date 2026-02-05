// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.util.ShootingOnTheFly;

public class RobotContainer {
  private final double m_maxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final double m_maxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric m_drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(m_maxSpeed * 0.1)
          .withRotationalDeadband(m_maxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final Telemetry m_logger = new Telemetry(m_maxSpeed);

  private final CommandXboxController m_joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  public final Shooter m_shooter = new Shooter();
  public final Spindexer m_spindexer = new Spindexer();
  public final Kicker m_kicker = new Kicker();
  public final Vision m_vision = new Vision(m_drivetrain::addVisionMeasurement);

  /* Path follower */
  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    m_autoChooser = AutoBuilder.buildAutoChooser("");
    SmartDashboard.putData("Auto Mode", m_autoChooser);
    configureBindings();
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(
            () ->
                m_drive
                    .withVelocityX(
                        -m_joystick.getLeftY()
                            * m_maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -m_joystick.getLeftX() * m_maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -m_joystick.getRightX()
                            * m_maxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    m_joystick.back().and(m_joystick.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    m_joystick.back().and(m_joystick.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    m_joystick
        .start()
        .and(m_joystick.y())
        .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    m_joystick
        .start()
        .and(m_joystick.x())
        .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    m_joystick.leftBumper().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

    // Shooter SysId routines - use POV buttons while holding back
    // Direction and mechanism are controlled by NetworkTables toggles:
    //   Shooter/SysId/Forward - true=forward, false=reverse
    //   Shooter/SysId/Roller - true=roller, false=main flywheel
    m_joystick.back().and(m_joystick.povUp()).whileTrue(m_shooter.sysIdQuasistatic());
    m_joystick.back().and(m_joystick.povDown()).whileTrue(m_shooter.sysIdDynamic());

    // Spin up shooter based on distance to goal while holding right bumper
    m_joystick
        .rightBumper()
        .whileTrue(
            m_shooter.spinUpForDistanceCommand(
                () -> {
                  Translation2d robotPosition = m_drivetrain.getState().Pose.getTranslation();
                  Translation2d goalPosition = Constants.FieldSpots.getHubPosition();
                  return robotPosition.getDistance(goalPosition);
                }));

    // Look at the hub while holding A button (flips based on alliance)
    m_joystick
        .a()
        .whileTrue(
            m_drivetrain.lookAtPoint(
                () -> Constants.FieldSpots.getHubPosition(),
                () -> -m_joystick.getLeftY() * m_maxSpeed,
                () -> -m_joystick.getLeftX() * m_maxSpeed,
                m_maxSpeed * 0.1));

    // SHOOTING ON THE FLY: Hold Y to spin up shooter with velocity compensation
    // and automatically aim at the SOTF-calculated point while driving
    m_joystick
        .y()
        .whileTrue(
            m_shooter
                .spinUpForSOTFCommand(
                    () -> m_drivetrain.getState().Pose,
                    () -> fieldRelativeSpeeds(),
                    () -> Constants.FieldSpots.getHubPosition())
                .alongWith(
                    m_drivetrain.lookAtPoint(
                        () ->
                            ShootingOnTheFly.calculateAimingPoint(
                                m_drivetrain.getState().Pose,
                                fieldRelativeSpeeds(),
                                Constants.FieldSpots.getHubPosition(),
                                Constants.ShooterConstants.kSOTFLatencyCompensation,
                                Constants.ShooterConstants.TIME_OF_FLIGHT_MAP),
                        () -> -m_joystick.getLeftY() * m_maxSpeed,
                        () -> -m_joystick.getLeftX() * m_maxSpeed,
                        m_maxSpeed * 0.1)));

    // Spindexer: spin while holding right trigger, stop on release
    m_joystick.rightTrigger().whileTrue(m_spindexer.runVoltageCommand(6.0));
    m_joystick.rightTrigger().onFalse(m_spindexer.stopCommand());

    m_drivetrain.registerTelemetry(m_logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return m_autoChooser.getSelected();
  }

  /**
   * Gets the current robot velocity in field-relative coordinates.
   *
   * @return Field-relative ChassisSpeeds.
   */
  private ChassisSpeeds fieldRelativeSpeeds() {
    ChassisSpeeds robotRelative = m_drivetrain.getState().Speeds;
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelative, m_drivetrain.getState().Pose.getRotation());
  }
}
