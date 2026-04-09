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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ShootOnTheMove;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FuelSim;
import frc.robot.util.FuelVisualizer;

public class RobotContainer {
  private final double m_maxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric m_drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(SwerveConstants.translationMaxSpeed.times(0.1))
          .withRotationalDeadband(m_maxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final Telemetry m_logger =
      new Telemetry(SwerveConstants.translationMaxSpeed.in(MetersPerSecond));

  /** Publisher for distance to goal (lookup table reference frame, in meters). */
  private final DoublePublisher m_distanceToGoalPub =
      NetworkTableInstance.getDefault().getDoubleTopic("Shooter/DistanceToGoal").publish();

  private final CommandXboxController m_joystick = new CommandXboxController(0);

  public final Swerve m_drivetrain = TunerConstants.createDrivetrain();
  public final Shooter m_shooter = new Shooter();
  public final Spindexer m_spindexer = new Spindexer();
  public final Kicker m_kicker = new Kicker();
  public final Intake m_intake = new Intake();
  public final Vision m_vision = new Vision(m_drivetrain::addVisionMeasurement);

  private static FuelSim m_fuelSim;

  /** Gets the fuel simulation instance. */
  public static FuelSim getFuelSim() {
    return m_fuelSim;
  }

  /* Path follower */
  private final SendableChooser<Command> m_autoChooser;

  private final PowerDistribution m_powerDistribution = new PowerDistribution();

  // Flag set by AimAndSpinUp when shooter reaches target speed
  private boolean m_autoReadyToShoot;

  // Dashboard toggle for shooting on the fly mode
  private final BooleanSubscriber m_sotfEnabledSub;
  private final Trigger m_sotfEnabledTrigger;

  // Dashboard toggle for horizontal pose mirroring
  private final BooleanSubscriber m_mirrorSub;

  public RobotContainer() {
    // Initialize mirror toggle on dashboard (before setupPathPlannerCommands which references it)
    SmartDashboard.putBoolean("Left Side", false);
    m_mirrorSub =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getBooleanTopic("Left Side")
            .subscribe(false);

    m_drivetrain.setShouldMirrorPath(m_mirrorSub::get);
    setupPathPlannerCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser("");
    SmartDashboard.putData("Auto Mode", m_autoChooser);

    // Initialize SOTF toggle on dashboard
    SmartDashboard.putBoolean("Shoot On The Move", true);
    m_sotfEnabledSub =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getBooleanTopic("Shoot On The Move")
            .subscribe(true);
    m_sotfEnabledTrigger = new Trigger(m_sotfEnabledSub::get);

    SmartDashboard.putData("Power Distribution", m_powerDistribution);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    configureBindings();
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    if (RobotBase.isSimulation()) {
      configureFuelSim();
    }
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
                            * SwerveConstants.translationMaxSpeed.in(
                                MetersPerSecond)) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -m_joystick.getLeftX()
                            * SwerveConstants.translationMaxSpeed.in(
                                MetersPerSecond)) // Drive left with negative X (left)
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
    //   Intake/Tuning/Enabled, RackPositionRotations, RackGains/*
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
                        () ->
                            -m_joystick.getLeftY()
                                * SwerveConstants.translationMaxSpeed.in(MetersPerSecond),
                        () ->
                            -m_joystick.getLeftX()
                                * SwerveConstants.translationMaxSpeed.in(MetersPerSecond),
                        SwerveConstants.translationMaxSpeed.times(0.1).in(MetersPerSecond))));

    // SOTF mode: spin up with velocity compensation, aim at SOTF-calculated point
    m_joystick
        .rightBumper()
        .and(m_sotfEnabledTrigger)
        .whileTrue(
            new ShootOnTheMove(
                () -> -m_joystick.getLeftY() * SwerveConstants.sotmMaxSpeed.in(MetersPerSecond),
                () -> -m_joystick.getLeftX() * SwerveConstants.sotmMaxSpeed.in(MetersPerSecond),
                SwerveConstants.sotmMaxSpeed.times(0.1).in(MetersPerSecond),
                m_drivetrain,
                m_shooter));

    // Idle shooter when not shooting and not aligning (bumper held keeps spin-up active)
    m_joystick
        .rightTrigger()
        .negate()
        .and(m_joystick.rightBumper().negate())
        .whileTrue(m_shooter.idleVoltage(2.0));

    Trigger actuallyShoot = m_joystick.rightTrigger().and(m_shooter.atSpeedTrigger());
    // Feed when shooter is at speed AND right trigger is held
    actuallyShoot.whileTrue(shootCommand());
    // Wiggle intake to feed balls while shooting, but only when not actively intaking
    actuallyShoot
        .and(m_joystick.leftTrigger().negate())
        .whileTrue(m_intake.feedingWiggleRackCommand());

    m_joystick.rightTrigger().whileFalse(Commands.parallel(m_spindexer.stop(), m_kicker.stop()));

    // Passing mode: spin up wheels to passing speed on left bumper hold, feed when at speed
    m_joystick.leftBumper().whileTrue(m_shooter.spinUpForPassingCommand());
    Trigger actuallyPass = m_joystick.leftBumper().and(m_shooter.atSpeedTrigger());
    actuallyPass.whileTrue(shootCommand());
    // Wiggle intake to feed balls while passing, but only when not actively intaking
    actuallyPass
        .and(m_joystick.leftTrigger().negate())
        .whileTrue(m_intake.feedingWiggleRackCommand());
    m_joystick.leftBumper().whileFalse(Commands.parallel(m_spindexer.stop(), m_kicker.stop()));

    // Reset field-centric heading on back button press
    m_joystick.back().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

    // Intake: deploy while holding left trigger, stow on A button
    m_joystick.leftTrigger().whileTrue(m_intake.deployCommand());
    m_joystick.a().onTrue(m_intake.stowCommand());
    m_joystick.x().onTrue(m_intake.deployOnly());
    m_joystick.y().whileTrue(m_intake.runRollersReverse());

    // Reverse commands
    m_joystick.povDown().whileTrue(m_kicker.spinReverse());
    m_joystick.povRight().whileTrue(m_spindexer.spinReverse());

    // GoToPose test: 10 ft forward, 4 ft right from current pose
    m_joystick
        .start()
        .onTrue(
            m_drivetrain.fastTransitRelative(
                Units.feetToMeters(10), // 10 ft forward
                Units.feetToMeters(-4))); // 4 ft right (negative = right)

    // Sim-only: set Sim/DisturbRobot to true in NetworkTables to teleport the robot off path.
    // Works during auto (joystick inputs are disabled during auto).
    // Auto-resets to false so you can trigger it repeatedly.
    if (RobotBase.isSimulation()) {
      var disturbTopic =
          NetworkTableInstance.getDefault().getTable("Sim").getBooleanTopic("DisturbRobot");
      var disturbPub = disturbTopic.publish();
      disturbPub.setDefault(false);
      BooleanSubscriber disturbSub = disturbTopic.subscribe(false);
      new Trigger(disturbSub::get)
          .onTrue(
              m_drivetrain
                  .simDisturbRobot()
                  .andThen(Commands.runOnce(() -> disturbPub.set(false))));
    }

    m_drivetrain.registerTelemetry(m_logger::telemeterize);
  }

  private void configureFuelSim() {
    m_fuelSim = new FuelSim();

    m_fuelSim.spawnStartingFuel();
    m_fuelSim.registerRobot(
        SwerveConstants.kRobotWidth,
        SwerveConstants.kRobotLength,
        Units.inchesToMeters(5),
        () -> m_drivetrain.getState().Pose,
        () -> m_drivetrain.getState().Speeds);

    m_fuelSim.start();

    // Intake is at the back after 180° front/back swap
    m_fuelSim.registerIntake(
        -SwerveConstants.kRobotLength / 2 - Units.inchesToMeters(12),
        -SwerveConstants.kRobotLength / 2,
        -SwerveConstants.kRobotWidth / 2,
        SwerveConstants.kRobotWidth / 2,
        m_intake.intakingTrigger(),
        FuelVisualizer::addFuel);
  }

  public Command alignToHub() {
    return m_drivetrain
        .lookAtPoint(
            m_drivetrain::getGoalAimPoint,
            () -> 0.0,
            () -> 0.0,
            SwerveConstants.translationMaxSpeed.in(MetersPerSecond) * 0.1)
        .until(
            () -> {
              Translation2d robotPosition = m_drivetrain.getState().Pose.getTranslation();
              Translation2d targetPoint = m_drivetrain.getGoalAimPoint();
              Rotation2d angleToTarget = targetPoint.minus(robotPosition).getAngle();
              Rotation2d robotHeading = m_drivetrain.getState().Pose.getRotation();
              double deg = robotHeading.minus(angleToTarget).getDegrees();
              return Math.abs(deg) <= 2.0;
            });
  }

  public Command shootCommand() {
    return Commands.parallel(
        m_spindexer.spin(),
        m_kicker.spin(),
        Commands.run(
            () -> {
              Translation2d robotPosition = m_drivetrain.getState().Pose.getTranslation();
              Translation2d hubPosition = Constants.FieldSpots.getHubPosition();
              double distance = robotPosition.getDistance(hubPosition);
              FuelVisualizer.trySpawnFuel(m_drivetrain.getState().Pose, hubPosition, distance);
            }));
  }

  public Command stopShooting() {
    return Commands.parallel(m_spindexer.stop(), m_kicker.stop(), m_intake.autoDeployCommand());
  }

  public Command doNothing() {
    return Commands.none();
  }

  public Command intake() {
    return m_intake.runRollers();
  }

  public void setupPathPlannerCommands() {
    NamedCommands.registerCommand("Shoot", shootCommand());
    NamedCommands.registerCommand("Intake", m_intake.runAutoRollers());
    NamedCommands.registerCommand("StopShooting", stopShooting());
    NamedCommands.registerCommand("AlignToHub", alignToHub());
    NamedCommands.registerCommand("IntakeDeploy", m_intake.deployOnly());
    NamedCommands.registerCommand("doNothing", doNothing());
    NamedCommands.registerCommand(
        "ResetOdomOverBump", m_drivetrain.resetRobotPoseOverBump(m_mirrorSub::get));
    NamedCommands.registerCommand(
        "ShooterWheelSpinUp", m_shooter.spinUpForDistanceCommand(() -> Units.feetToMeters(9.5)));
    NamedCommands.registerCommand(
        "SpinUp",
        m_shooter
            .spinUpForDistanceCommand(m_drivetrain::getLookupDistanceToGoal)
            .until(m_shooter.atSpeedTrigger()));
    NamedCommands.registerCommand(
        "AimAndSpinUp",
        Commands.runOnce(() -> m_autoReadyToShoot = false)
            .andThen(
                m_drivetrain
                    .aimAtHubDuringPath()
                    .alongWith(
                        m_shooter.spinUpForSOTFCommand(
                            () -> m_drivetrain.getState().Pose,
                            this::fieldRelativeSpeeds,
                            this::fieldRelativeAccel,
                            Constants.FieldSpots::getHubPosition),
                        Commands.run(
                            () -> m_autoReadyToShoot = m_shooter.atSpeedTrigger().getAsBoolean())))
            .finallyDo(() -> m_autoReadyToShoot = false));
    NamedCommands.registerCommand(
        "ShootWhenReady", Commands.waitUntil(() -> m_autoReadyToShoot).andThen(shootCommand()));
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

  private ChassisSpeeds fieldRelativeAccel() {
    ChassisSpeeds robotRelative = m_drivetrain.getAcceleration();
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelative, m_drivetrain.getState().Pose.getRotation());
  }
}
