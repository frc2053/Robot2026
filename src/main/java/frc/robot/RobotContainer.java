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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.util.FuelVisualizer;
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

  /** Publisher for distance to goal (lookup table reference frame, in meters). */
  private final DoublePublisher m_distanceToGoalPub =
      NetworkTableInstance.getDefault().getDoubleTopic("Shooter/DistanceToGoal").publish();

  private final CommandXboxController m_joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  public final Shooter m_shooter = new Shooter();
  public final Spindexer m_spindexer = new Spindexer();
  public final Kicker m_kicker = new Kicker();
  public final Intake m_intake = new Intake();
  public final Climber m_climber = new Climber();
  public final Vision m_vision = new Vision(m_drivetrain::addVisionMeasurement);

  /* Path follower */
  private final SendableChooser<Command> m_autoChooser;

  // Dashboard toggle for shooting on the fly mode
  private final BooleanSubscriber m_sotfEnabledSub;
  private final Trigger m_sotfEnabledTrigger;

  public RobotContainer() {
    m_autoChooser = AutoBuilder.buildAutoChooser("");
    SmartDashboard.putData("Auto Mode", m_autoChooser);

    // Initialize SOTF toggle on dashboard
    SmartDashboard.putBoolean("Shoot On The Move", false);
    m_sotfEnabledSub =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getBooleanTopic("Shoot On The Move")
            .subscribe(false);
    m_sotfEnabledTrigger = new Trigger(m_sotfEnabledSub::get);

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

    // Tuning commands - controlled via NetworkTables Enabled entry:
    //   Shooter/Tuning/Enabled, MainShooterRPM, RollerRPM, MainShooterGains/*, RollerGains/*
    //   Intake/Tuning/Enabled, PivotPositionRotations, PivotGains/*
    m_shooter.tuningEnabledTrigger().whileTrue(m_shooter.tuningCommand());
    m_intake.tuningEnabledTrigger().whileTrue(m_intake.tuningCommand());

    // Left trigger: Shoot (spin up, aim)
    // Dashboard toggle "Shoot On The Move" switches between static and SOTF modes
    // Static mode: spin up based on distance, aim at hub
    m_joystick
        .rightBumper()
        .and(m_sotfEnabledTrigger.negate())
        .whileTrue(
            m_shooter
                .spinUpForDistanceCommand(
                    () -> {
                      double lookupDistance = m_drivetrain.getLookupDistanceToGoal();
                      m_distanceToGoalPub.set(lookupDistance);
                      return lookupDistance;
                    })
                .alongWith(
                    m_drivetrain.lookAtPoint(
                        m_drivetrain::getGoalAimPoint,
                        () -> -m_joystick.getLeftY() * m_maxSpeed,
                        () -> -m_joystick.getLeftX() * m_maxSpeed,
                        m_maxSpeed * 0.1)));

    // SOTF mode: spin up with velocity compensation, aim at SOTF-calculated point
    m_joystick
        .rightBumper()
        .and(m_sotfEnabledTrigger)
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

    // Idle shooter when not shooting
    m_joystick.rightTrigger().whileFalse(m_shooter.idleVoltage(2.0));

    Trigger actuallyShoot = m_joystick.rightTrigger().and(m_shooter.atSpeedTrigger());
    // Feed when shooter is at speed AND right trigger is held
    actuallyShoot.whileTrue(shootCommand());

    m_joystick.rightTrigger().whileFalse(Commands.parallel(m_spindexer.stop(), m_kicker.stop()));

    // Reset field-centric heading on back button press
    m_joystick.back().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

    // Intake: deploy while holding left trigger, stow on left bumper
    m_joystick.leftTrigger().whileTrue(m_intake.deployCommand());
    m_joystick.a().onTrue(m_intake.stowCommand());
    m_joystick.x().onTrue(m_intake.deployOnly());
    m_joystick.y().whileTrue(m_intake.runRollersReverse());

    // Reverse commands
    m_joystick.povDown().whileTrue(m_kicker.spinReverse());
    m_joystick.povUp().whileTrue(m_spindexer.spinReverse());

    // Climber: D-pad up toggles between extend and retract
    m_joystick
        .povUp()
        .onTrue(
            Commands.either(
                m_climber.retractCommand().until(m_climber::atPosition),
                m_climber.extendCommand().until(m_climber::atPosition),
                m_climber::isExtended));

    m_drivetrain.registerTelemetry(m_logger::telemeterize);
  }

  public Command alignToHub() {
    return m_drivetrain
        .lookAtPoint(m_drivetrain::getGoalAimPoint, () -> 0.0, () -> 0.0, m_maxSpeed * 0.1)
        .until(
            () -> {
              Translation2d robotPosition = m_drivetrain.getState().Pose.getTranslation();
              Translation2d targetPoint = m_drivetrain.getGoalAimPoint();
              Rotation2d angleToTarget = targetPoint.minus(robotPosition).getAngle();
              Rotation2d robotHeading = m_drivetrain.getState().Pose.getRotation();
              return Math.abs(robotHeading.minus(angleToTarget).getDegrees()) <= 2.0;
            });
  }

  public Command shootCommand() {
    return Commands.parallel(
        m_spindexer.spin(),
        m_kicker.spin(),
        m_intake.feedingWigglePivotCommand(),
        Commands.run(
            () -> {
              Translation2d robotPosition = m_drivetrain.getState().Pose.getTranslation();
              Translation2d hubPosition = Constants.FieldSpots.getHubPosition();
              double distance = robotPosition.getDistance(hubPosition);
              FuelVisualizer.trySpawnFuel(m_drivetrain.getState().Pose, hubPosition, distance);
            }));
  }

  public Command stopShooting() {
    return Commands.parallel(m_spindexer.stop(), m_kicker.stop(), m_intake.deployCommand());
  }

  public void setupPathPlannerCommands() {
    NamedCommands.registerCommand("Shoot", shootCommand());
    NamedCommands.registerCommand("StopShooting", stopShooting());
    NamedCommands.registerCommand("AlignToHub", alignToHub());
    NamedCommands.registerCommand("IntakeDeploy", m_intake.deployCommand());
    NamedCommands.registerCommand(
        "ShooterWheelSpinUp", m_shooter.spinUpForDistanceCommand(() -> Units.feetToMeters(9.5)));
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
