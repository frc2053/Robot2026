// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ShootingOnTheFly;
import frc.robot.util.ShootingOnTheFly.SOTFResult;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {

  private final TalonFX m_shooterMotorLeft;
  private final TalonFX m_shooterMotorRight;
  private final TalonFX m_shooterMotorTopRoller;

  private final StatusSignal<AngularVelocity> m_leftMotorVel;
  private final StatusSignal<AngularVelocity> m_rightMotorVel;
  private final StatusSignal<AngularVelocity> m_rollerVel;

  private final StatusSignal<Angle> m_leftMotorPos;
  private final StatusSignal<Angle> m_rightMotorPos;
  private final StatusSignal<Angle> m_rollerPos;

  private final StatusSignal<Voltage> m_leftMotorVoltage;
  private final StatusSignal<Voltage> m_rightMotorVoltage;
  private final StatusSignal<Voltage> m_rollerVoltage;

  private final StatusSignal<Current> m_leftMotorStatorCurrent;
  private final StatusSignal<Current> m_rightMotorStatorCurrent;
  private final StatusSignal<Current> m_rollerStatorCurrent;
  private final StatusSignal<Current> m_leftMotorSupplyCurrent;
  private final StatusSignal<Current> m_rightMotorSupplyCurrent;
  private final StatusSignal<Current> m_rollerSupplyCurrent;

  // Simulation objects
  private final TalonFXSimState m_leftMotorSimState;
  private final TalonFXSimState m_rightMotorSimState;
  private final TalonFXSimState m_rollerSimState;
  private final DCMotorSim m_mainShooterSim;
  private final DCMotorSim m_rollerSim;

  // NetworkTables publishers for logging
  private final DoublePublisher m_leftMotorVelPub;
  private final DoublePublisher m_rightMotorVelPub;
  private final DoublePublisher m_rollerVelPub;
  private final DoublePublisher m_leftMotorVoltagePub;
  private final DoublePublisher m_rightMotorVoltagePub;
  private final DoublePublisher m_rollerVoltagePub;
  private final DoublePublisher m_leftMotorStatorCurrentPub;
  private final DoublePublisher m_rightMotorStatorCurrentPub;
  private final DoublePublisher m_rollerStatorCurrentPub;
  private final DoublePublisher m_leftMotorSupplyCurrentPub;
  private final DoublePublisher m_rightMotorSupplyCurrentPub;
  private final DoublePublisher m_rollerSupplyCurrentPub;
  private final StringPublisher m_currentCommandPub;
  private final BooleanPublisher m_atSpeedPub;
  private final DoublePublisher m_mainShooterSetpointPub;
  private final DoublePublisher m_rollerSetpointPub;

  // Target velocities for at-speed detection
  private double m_targetMainShooterRps;
  private double m_targetRollerRps;

  // Public trigger for when shooter is at speed
  private final Trigger m_atSpeedTrigger;

  // Control requests
  private final MotionMagicVelocityVoltage m_mainShooterVelocityRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final MotionMagicVelocityVoltage m_rollerVelocityRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final VoltageOut m_voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  // SysId routine for main shooter flywheel (averages both motors)
  private final SysIdRoutine m_mainShooterSysId;
  // SysId routine for top roller
  private final SysIdRoutine m_rollerSysId;

  // SysId NetworkTables controls - just toggle switches for direction and mechanism selection
  private final BooleanSubscriber m_sysIdForwardSub;
  private final BooleanSubscriber m_sysIdRollerSub;
  private final BooleanPublisher m_sysIdForwardPub;
  private final BooleanPublisher m_sysIdRollerPub;

  // Tuning mode NetworkTables controls
  private final BooleanSubscriber m_tuningEnabledSub;
  private final BooleanPublisher m_tuningEnabledPub;
  private final DoubleSubscriber m_tuningMainShooterRpsSub;
  private final DoublePublisher m_tuningMainShooterRpsPub;
  private final DoubleSubscriber m_tuningRollerRpsSub;
  private final DoublePublisher m_tuningRollerRpsPub;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterMotorLeft = new TalonFX(ShooterConstants.SHOOTER_MOTOR_LEFT_ID);
    m_shooterMotorRight = new TalonFX(ShooterConstants.SHOOTER_MOTOR_RIGHT_ID);
    m_shooterMotorTopRoller = new TalonFX(ShooterConstants.SHOOTER_MOTOR_TOP_ROLLER_ID);

    TalonFXConfiguration mainShooterConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(ShooterConstants.SHOOTER_STATOR_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ShooterConstants.SHOOTER_SUPPLY_LIMIT))
            .withSlot0(
                new Slot0Configs()
                    .withKS(ShooterConstants.kMainShooterKS)
                    .withKV(ShooterConstants.kMainShooterKV)
                    .withKA(ShooterConstants.kMainShooterKA)
                    .withKP(ShooterConstants.kMainShooterKP)
                    .withKI(ShooterConstants.kMainShooterKI)
                    .withKD(ShooterConstants.kMainShooterKD))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(ShooterConstants.kMainShooterMotionMagicAccel));

    // Roller config with different PID gains and inverted direction (clockwise positive)
    TalonFXConfiguration rollerConfig =
        mainShooterConfig
            .clone()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(
                new Slot0Configs()
                    .withKS(ShooterConstants.kRollerKS)
                    .withKV(ShooterConstants.kRollerKV)
                    .withKA(ShooterConstants.kRollerKA)
                    .withKP(ShooterConstants.kRollerKP)
                    .withKI(ShooterConstants.kRollerKI)
                    .withKD(ShooterConstants.kRollerKD))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(ShooterConstants.kMainShooterMotionMagicAccel));

    StatusCode shooterLeftConfigResult =
        m_shooterMotorLeft.getConfigurator().apply(mainShooterConfig);
    if (!shooterLeftConfigResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to left shooter motor!");
    }
    StatusCode shooterRightConfigResult =
        m_shooterMotorRight.getConfigurator().apply(mainShooterConfig);
    if (!shooterRightConfigResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to right shooter motor!");
    }
    StatusCode rollerConfigResult = m_shooterMotorTopRoller.getConfigurator().apply(rollerConfig);
    if (!rollerConfigResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to roller shooter motor!");
    }

    m_leftMotorVel = m_shooterMotorLeft.getVelocity();
    m_rightMotorVel = m_shooterMotorRight.getVelocity();
    m_rollerVel = m_shooterMotorTopRoller.getVelocity();

    m_leftMotorPos = m_shooterMotorLeft.getPosition();
    m_rightMotorPos = m_shooterMotorRight.getPosition();
    m_rollerPos = m_shooterMotorTopRoller.getPosition();

    m_leftMotorVoltage = m_shooterMotorLeft.getMotorVoltage();
    m_rightMotorVoltage = m_shooterMotorRight.getMotorVoltage();
    m_rollerVoltage = m_shooterMotorTopRoller.getMotorVoltage();

    m_leftMotorStatorCurrent = m_shooterMotorLeft.getStatorCurrent();
    m_rightMotorStatorCurrent = m_shooterMotorRight.getStatorCurrent();
    m_rollerStatorCurrent = m_shooterMotorTopRoller.getStatorCurrent();
    m_leftMotorSupplyCurrent = m_shooterMotorLeft.getSupplyCurrent();
    m_rightMotorSupplyCurrent = m_shooterMotorRight.getSupplyCurrent();
    m_rollerSupplyCurrent = m_shooterMotorTopRoller.getSupplyCurrent();

    StatusCode setUpdateFreqResult =
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            m_leftMotorVel,
            m_rightMotorVel,
            m_rollerVel,
            m_leftMotorPos,
            m_rightMotorPos,
            m_rollerPos,
            m_leftMotorVoltage,
            m_rightMotorVoltage,
            m_rollerVoltage,
            m_leftMotorStatorCurrent,
            m_rightMotorStatorCurrent,
            m_rollerStatorCurrent,
            m_leftMotorSupplyCurrent,
            m_rightMotorSupplyCurrent,
            m_rollerSupplyCurrent);
    if (!setUpdateFreqResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply update frequency for shooter subsystem!");
    }
    StatusCode optiResult =
        ParentDevice.optimizeBusUtilizationForAll(
            m_shooterMotorLeft, m_shooterMotorRight, m_shooterMotorTopRoller);
    if (!optiResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply optimization for shooter subsystem!");
    }

    // Set left motor to follow right motor (they are mechanically coupled)
    // MotorAlignmentValue.Aligned because they spin in the same direction
    m_shooterMotorLeft.setControl(
        new Follower(ShooterConstants.SHOOTER_MOTOR_RIGHT_ID, MotorAlignmentValue.Aligned));

    // Initialize NetworkTables publishers for logging
    NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("Shooter");
    m_leftMotorVelPub = shooterTable.getDoubleTopic("LeftMotorVelocityRPM").publish();
    m_rightMotorVelPub = shooterTable.getDoubleTopic("RightMotorVelocityRPM").publish();
    m_rollerVelPub = shooterTable.getDoubleTopic("RollerVelocityRPM").publish();
    m_leftMotorVoltagePub = shooterTable.getDoubleTopic("LeftMotorVoltage").publish();
    m_rightMotorVoltagePub = shooterTable.getDoubleTopic("RightMotorVoltage").publish();
    m_rollerVoltagePub = shooterTable.getDoubleTopic("RollerVoltage").publish();
    m_leftMotorStatorCurrentPub = shooterTable.getDoubleTopic("LeftMotorStatorCurrent").publish();
    m_rightMotorStatorCurrentPub = shooterTable.getDoubleTopic("RightMotorStatorCurrent").publish();
    m_rollerStatorCurrentPub = shooterTable.getDoubleTopic("RollerStatorCurrent").publish();
    m_leftMotorSupplyCurrentPub = shooterTable.getDoubleTopic("LeftMotorSupplyCurrent").publish();
    m_rightMotorSupplyCurrentPub = shooterTable.getDoubleTopic("RightMotorSupplyCurrent").publish();
    m_rollerSupplyCurrentPub = shooterTable.getDoubleTopic("RollerSupplyCurrent").publish();
    m_currentCommandPub = shooterTable.getStringTopic("CurrentCommand").publish();
    m_atSpeedPub = shooterTable.getBooleanTopic("AtSpeed").publish();
    m_mainShooterSetpointPub = shooterTable.getDoubleTopic("MainShooterSetpointRPM").publish();
    m_rollerSetpointPub = shooterTable.getDoubleTopic("RollerSetpointRPM").publish();

    // Initialize at-speed trigger
    m_atSpeedTrigger = new Trigger(this::isAtSpeed);

    // Initialize simulation
    m_leftMotorSimState = m_shooterMotorLeft.getSimState();
    m_rightMotorSimState = m_shooterMotorRight.getSimState();
    m_rollerSimState = m_shooterMotorTopRoller.getSimState();

    // Set motor types for accurate simulation
    m_leftMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    m_rightMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    m_rollerSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    // Main shooter flywheel sim (powered by left and right motors)
    // Using 2 Kraken X60 motors
    m_mainShooterSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(2),
                ShooterConstants.MAIN_SHOOTER_MOI,
                ShooterConstants.MAIN_SHOOTER_GEAR_RATIO),
            DCMotor.getKrakenX60(2));

    // Roller flywheel sim (powered by single motor)
    m_rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                ShooterConstants.ROLLER_MOI,
                ShooterConstants.ROLLER_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    // SysId routine for main shooter flywheel
    // Both motors are applied the same voltage, and we log the average velocity
    m_mainShooterSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                edu.wpi.first.units.Units.Volts.of(10), // Dynamic step voltage
                null, // Use default timeout (10 s)
                state -> SignalLogger.writeString("ShooterFlywheelSysId_State", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> {
                  // Left motor follows right motor automatically
                  m_shooterMotorRight.setControl(m_voltageRequest.withOutput(volts.in(Volts)));
                },
                null, // Use SignalLogger for logging
                this));

    // SysId routine for top roller
    m_rollerSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                edu.wpi.first.units.Units.Volts.of(10),
                null,
                state -> SignalLogger.writeString("ShooterRollerSysId_State", state.toString())),
            new SysIdRoutine.Mechanism(
                volts ->
                    m_shooterMotorTopRoller.setControl(
                        m_voltageRequest.withOutput(volts.in(Volts))),
                null,
                this));

    // Initialize SysId NetworkTables controls
    NetworkTable sysIdTable = shooterTable.getSubTable("SysId");

    // Toggle switches for direction and mechanism selection
    m_sysIdForwardPub = sysIdTable.getBooleanTopic("Forward").publish();
    m_sysIdForwardSub = sysIdTable.getBooleanTopic("Forward").subscribe(true);
    m_sysIdForwardPub.set(true); // Default to forward

    m_sysIdRollerPub = sysIdTable.getBooleanTopic("Roller").publish();
    m_sysIdRollerSub = sysIdTable.getBooleanTopic("Roller").subscribe(false);
    m_sysIdRollerPub.set(false); // Default to main flywheel

    // Initialize tuning mode NetworkTables controls
    NetworkTable tuningTable = shooterTable.getSubTable("Tuning");
    m_tuningEnabledPub = tuningTable.getBooleanTopic("Enabled").publish();
    m_tuningEnabledSub = tuningTable.getBooleanTopic("Enabled").subscribe(false);
    m_tuningEnabledPub.set(false);
    m_tuningMainShooterRpsPub = tuningTable.getDoubleTopic("MainShooterRPM").publish();
    m_tuningMainShooterRpsSub = tuningTable.getDoubleTopic("MainShooterRPM").subscribe(0.0);
    m_tuningMainShooterRpsPub.set(0.0);
    m_tuningRollerRpsPub = tuningTable.getDoubleTopic("RollerRPM").publish();
    m_tuningRollerRpsSub = tuningTable.getDoubleTopic("RollerRPM").subscribe(0.0);
    m_tuningRollerRpsPub.set(0.0);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        m_leftMotorVel,
        m_rightMotorVel,
        m_rollerVel,
        m_leftMotorVoltage,
        m_rightMotorVoltage,
        m_rollerVoltage,
        m_leftMotorStatorCurrent,
        m_rightMotorStatorCurrent,
        m_rollerStatorCurrent,
        m_leftMotorSupplyCurrent,
        m_rightMotorSupplyCurrent,
        m_rollerSupplyCurrent);

    // Publish velocity signals to NetworkTables in RPM
    m_leftMotorVelPub.set(m_leftMotorVel.getValue().in(RotationsPerSecond) * 60.0);
    m_rightMotorVelPub.set(m_rightMotorVel.getValue().in(RotationsPerSecond) * 60.0);
    m_rollerVelPub.set(m_rollerVel.getValue().in(RotationsPerSecond) * 60.0);

    // Publish voltage signals to NetworkTables
    m_leftMotorVoltagePub.set(m_leftMotorVoltage.getValue().in(Volts));
    m_rightMotorVoltagePub.set(m_rightMotorVoltage.getValue().in(Volts));
    m_rollerVoltagePub.set(m_rollerVoltage.getValue().in(Volts));

    // Publish current signals to NetworkTables
    m_leftMotorStatorCurrentPub.set(m_leftMotorStatorCurrent.getValue().in(Amps));
    m_rightMotorStatorCurrentPub.set(m_rightMotorStatorCurrent.getValue().in(Amps));
    m_rollerStatorCurrentPub.set(m_rollerStatorCurrent.getValue().in(Amps));
    m_leftMotorSupplyCurrentPub.set(m_leftMotorSupplyCurrent.getValue().in(Amps));
    m_rightMotorSupplyCurrentPub.set(m_rightMotorSupplyCurrent.getValue().in(Amps));
    m_rollerSupplyCurrentPub.set(m_rollerSupplyCurrent.getValue().in(Amps));

    // Publish current command name
    Command currentCommand = getCurrentCommand();
    m_currentCommandPub.set(currentCommand != null ? currentCommand.getName() : "None");

    // Publish at-speed status
    m_atSpeedPub.set(isAtSpeed());

    // Publish velocity setpoints in RPM
    m_mainShooterSetpointPub.set(m_targetMainShooterRps * 60.0);
    m_rollerSetpointPub.set(m_targetRollerRps * 60.0);
  }

  @Override
  public void simulationPeriodic() {
    // Update supply voltage from battery
    m_leftMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_rightMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_rollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Main shooter flywheel: average voltage from left and right motors
    double mainShooterVoltage =
        (m_leftMotorSimState.getMotorVoltageMeasure().in(Volts)
                + m_rightMotorSimState.getMotorVoltageMeasure().in(Volts))
            / 2.0;
    m_mainShooterSim.setInputVoltage(mainShooterVoltage);
    m_mainShooterSim.update(0.020);

    // Update left and right motor sim states with rotor position and velocity
    // Note: DCMotorSim returns mechanism values, multiply by gear ratio for rotor values
    m_leftMotorSimState.setRawRotorPosition(
        m_mainShooterSim.getAngularPosition().times(ShooterConstants.MAIN_SHOOTER_GEAR_RATIO));
    m_leftMotorSimState.setRotorVelocity(
        m_mainShooterSim.getAngularVelocity().times(ShooterConstants.MAIN_SHOOTER_GEAR_RATIO));
    m_rightMotorSimState.setRawRotorPosition(
        m_mainShooterSim.getAngularPosition().times(ShooterConstants.MAIN_SHOOTER_GEAR_RATIO));
    m_rightMotorSimState.setRotorVelocity(
        m_mainShooterSim.getAngularVelocity().times(ShooterConstants.MAIN_SHOOTER_GEAR_RATIO));

    // Roller flywheel
    m_rollerSim.setInputVoltage(m_rollerSimState.getMotorVoltageMeasure().in(Volts));
    m_rollerSim.update(0.020);

    // Update roller motor sim state with rotor position and velocity
    m_rollerSimState.setRawRotorPosition(
        m_rollerSim.getAngularPosition().times(ShooterConstants.ROLLER_GEAR_RATIO));
    m_rollerSimState.setRotorVelocity(
        m_rollerSim.getAngularVelocity().times(ShooterConstants.ROLLER_GEAR_RATIO));
  }

  /**
   * Returns whether the shooter is at the target speed.
   *
   * @return true if both main shooter and roller are within tolerance of target speed
   */
  public boolean isAtSpeed() {
    double mainShooterVel = m_rightMotorVel.getValue().in(RotationsPerSecond);
    double rollerVel = m_rollerVel.getValue().in(RotationsPerSecond);
    boolean mainAtSpeed =
        Math.abs(mainShooterVel - m_targetMainShooterRps) < ShooterConstants.kVelocityToleranceRps;
    boolean rollerAtSpeed =
        Math.abs(rollerVel - m_targetRollerRps) < ShooterConstants.kVelocityToleranceRps;
    return mainAtSpeed && rollerAtSpeed;
  }

  /**
   * Returns a trigger that is true when the shooter is at the target speed.
   *
   * @return the at-speed trigger
   */
  public Trigger atSpeedTrigger() {
    return m_atSpeedTrigger;
  }

  /**
   * Creates a command that spins up the shooter wheels based on distance to goal. Uses
   * interpolating tree maps to determine the appropriate speeds for the bottom shooter and top
   * roller based on the supplied distance.
   *
   * @param distanceSupplier A supplier that provides the distance to goal in meters
   * @return A command that runs the shooter at the interpolated speeds
   */
  public Command spinUpForDistanceCommand(DoubleSupplier distanceSupplier) {
    return this.run(
            () -> {
              double distance = distanceSupplier.getAsDouble();

              // Look up speeds from interpolating tree maps
              double bottomSpeedRps = ShooterConstants.BOTTOM_SHOOTER_SPEED_MAP.get(distance);
              double topRollerSpeedRps = ShooterConstants.TOP_ROLLER_SPEED_MAP.get(distance);

              // Store target velocities for at-speed detection
              m_targetMainShooterRps = bottomSpeedRps;
              m_targetRollerRps = topRollerSpeedRps;

              // Command the bottom shooter (right motor is leader, left follows)
              m_shooterMotorRight.setControl(
                  m_mainShooterVelocityRequest.withVelocity(bottomSpeedRps));

              // Command the top roller
              m_shooterMotorTopRoller.setControl(
                  m_rollerVelocityRequest.withVelocity(topRollerSpeedRps));
            })
        .withName("SpinUpForDistance");
  }

  public Command idleVoltae(double idleVoltage) {
    return this.run(
            () -> {
              m_targetMainShooterRps = 0.0;
              m_targetRollerRps = 0.0;

              // Set bottom shooter motor (leader)
              m_shooterMotorRight.setControl(m_voltageRequest.withOutput(idleVoltage));

              // Set top roller motor
              m_shooterMotorTopRoller.setControl(m_voltageRequest.withOutput(idleVoltage));
            })
        .withName("IdleShooterLowerVoltage");
  }

  /**
   * Creates a command that spins up the shooter using Shooting On The Fly (SOTF) calculations. This
   * compensates for robot velocity to ensure accurate shots while moving.
   *
   * @param robotPoseSupplier Supplier for current robot pose
   * @param robotSpeedsSupplier Supplier for current robot velocity (field-relative)
   * @param goalPositionSupplier Supplier for the goal position to shoot at
   * @return A command that runs the shooter at SOTF-adjusted speeds
   */
  public Command spinUpForSOTFCommand(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotSpeedsSupplier,
      Supplier<Translation2d> goalPositionSupplier) {
    return this.run(
            () -> {
              Pose2d robotPose = robotPoseSupplier.get();
              ChassisSpeeds robotSpeeds = robotSpeedsSupplier.get();
              Translation2d goalPosition = goalPositionSupplier.get();

              // Calculate SOTF-adjusted parameters
              SOTFResult result =
                  ShootingOnTheFly.calculate(
                      robotPose,
                      robotSpeeds,
                      goalPosition,
                      ShooterConstants.kSOTFLatencyCompensation,
                      ShooterConstants.TIME_OF_FLIGHT_MAP,
                      ShooterConstants.kMaxHorizontalVelocity);

              // Use the effective distance to look up RPM from our tuned tables
              double effectiveDistance = result.effectiveDistance();

              // Look up speeds from interpolating tree maps using effective distance
              double bottomSpeedRps =
                  ShooterConstants.BOTTOM_SHOOTER_SPEED_MAP.get(effectiveDistance);
              double topRollerSpeedRps =
                  ShooterConstants.TOP_ROLLER_SPEED_MAP.get(effectiveDistance);

              // Store target velocities for at-speed detection
              m_targetMainShooterRps = bottomSpeedRps;
              m_targetRollerRps = topRollerSpeedRps;

              // Command the bottom shooter (right motor is leader, left follows)
              m_shooterMotorRight.setControl(
                  m_mainShooterVelocityRequest.withVelocity(bottomSpeedRps));

              // Command the top roller
              m_shooterMotorTopRoller.setControl(
                  m_rollerVelocityRequest.withVelocity(topRollerSpeedRps));
            })
        .withName("SpinUpForSOTF");
  }

  /**
   * Creates a command that stops all shooter motors.
   *
   * @return A command that stops the shooter
   */
  public Command stopCommand() {
    return this.runOnce(
            () -> {
              m_targetMainShooterRps = 0.0;
              m_targetRollerRps = 0.0;
              m_shooterMotorRight.setControl(m_neutralRequest);
              m_shooterMotorTopRoller.setControl(m_neutralRequest);
            })
        .withName("StopShooter");
  }

  /**
   * Creates a command for tuning mode. When the Shooter/Tuning/Enabled toggle is true, the shooter
   * runs at the RPM values from Shooter/Tuning/MainShooterRPM and Shooter/Tuning/RollerRPM
   * (converted to RPS). When the toggle is false, the motors are stopped.
   *
   * @return A command that runs the shooter in tuning mode.
   */
  public Command tuningCommand() {
    return this.run(
            () -> {
              if (m_tuningEnabledSub.get()) {
                double mainRps = m_tuningMainShooterRpsSub.get() / 60.0;
                double rollerRps = m_tuningRollerRpsSub.get() / 60.0;
                m_targetMainShooterRps = mainRps;
                m_targetRollerRps = rollerRps;
                m_shooterMotorRight.setControl(m_mainShooterVelocityRequest.withVelocity(mainRps));
                m_shooterMotorTopRoller.setControl(m_rollerVelocityRequest.withVelocity(rollerRps));
              } else {
                m_targetMainShooterRps = 0.0;
                m_targetRollerRps = 0.0;
                m_shooterMotorRight.setControl(m_neutralRequest);
                m_shooterMotorTopRoller.setControl(m_neutralRequest);
              }
            })
        .withName("ShooterTuning");
  }

  /**
   * Creates a quasistatic SysId command for the main shooter flywheel. Use this to characterize the
   * shooter's kS and kV gains.
   *
   * @param direction The direction to run the quasistatic test
   * @return A command that runs the quasistatic test
   */
  public Command sysIdQuasistaticMainShooter(SysIdRoutine.Direction direction) {
    return m_mainShooterSysId.quasistatic(direction);
  }

  /**
   * Creates a dynamic SysId command for the main shooter flywheel. Use this to characterize the
   * shooter's kA gain.
   *
   * @param direction The direction to run the dynamic test
   * @return A command that runs the dynamic test
   */
  public Command sysIdDynamicMainShooter(SysIdRoutine.Direction direction) {
    return m_mainShooterSysId.dynamic(direction);
  }

  /**
   * Creates a quasistatic SysId command for the top roller. Use this to characterize the roller's
   * kS and kV gains.
   *
   * @param direction The direction to run the quasistatic test
   * @return A command that runs the quasistatic test
   */
  public Command sysIdQuasistaticRoller(SysIdRoutine.Direction direction) {
    return m_rollerSysId.quasistatic(direction);
  }

  /**
   * Creates a dynamic SysId command for the top roller. Use this to characterize the roller's kA
   * gain.
   *
   * @param direction The direction to run the dynamic test
   * @return A command that runs the dynamic test
   */
  public Command sysIdDynamicRoller(SysIdRoutine.Direction direction) {
    return m_rollerSysId.dynamic(direction);
  }

  // =============================================================================
  // SysId Commands using NetworkTables toggles
  // =============================================================================

  /**
   * Creates a quasistatic SysId command using NT toggle settings. Direction is determined by
   * Shooter/SysId/Forward toggle. Mechanism is determined by Shooter/SysId/Roller toggle.
   *
   * @return A command that runs the quasistatic test
   */
  public Command sysIdQuasistatic() {
    return Commands.either(
        Commands.either(
            sysIdQuasistaticRoller(SysIdRoutine.Direction.kForward),
            sysIdQuasistaticMainShooter(SysIdRoutine.Direction.kForward),
            () -> m_sysIdRollerSub.get()),
        Commands.either(
            sysIdQuasistaticRoller(SysIdRoutine.Direction.kReverse),
            sysIdQuasistaticMainShooter(SysIdRoutine.Direction.kReverse),
            () -> m_sysIdRollerSub.get()),
        () -> m_sysIdForwardSub.get());
  }

  /**
   * Creates a dynamic SysId command using NT toggle settings. Direction is determined by
   * Shooter/SysId/Forward toggle. Mechanism is determined by Shooter/SysId/Roller toggle.
   *
   * @return A command that runs the dynamic test
   */
  public Command sysIdDynamic() {
    return Commands.either(
        Commands.either(
            sysIdDynamicRoller(SysIdRoutine.Direction.kForward),
            sysIdDynamicMainShooter(SysIdRoutine.Direction.kForward),
            () -> m_sysIdRollerSub.get()),
        Commands.either(
            sysIdDynamicRoller(SysIdRoutine.Direction.kReverse),
            sysIdDynamicMainShooter(SysIdRoutine.Direction.kReverse),
            () -> m_sysIdRollerSub.get()),
        () -> m_sysIdForwardSub.get());
  }
}
