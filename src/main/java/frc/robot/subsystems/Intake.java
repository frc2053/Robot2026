// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.MechanismVisualizer;

/** Intake subsystem with a rack and roller motor. */
public class Intake extends SubsystemBase {

  private final TalonFX m_rackMotor;
  private final TalonFX m_rollerMotor;

  // Rack status signals
  private final StatusSignal<Angle> m_rackPosition;
  private final StatusSignal<AngularVelocity> m_rackVelocity;
  private final StatusSignal<Voltage> m_rackVoltage;

  // Roller status signals
  private final StatusSignal<AngularVelocity> m_rollerVelocity;
  private final StatusSignal<Voltage> m_rollerVoltage;

  // Current status signals
  private final StatusSignal<Current> m_rackStatorCurrent;
  private final StatusSignal<Current> m_rackSupplyCurrent;
  private final StatusSignal<Current> m_rollerStatorCurrent;
  private final StatusSignal<Current> m_rollerSupplyCurrent;

  // Simulation objects
  private final TalonFXSimState m_rackSimState;
  private final TalonFXSimState m_rollerSimState;
  private final ElevatorSim m_rackSim;
  private final DCMotorSim m_rollerSim;

  // NetworkTables publishers for logging
  private final DoublePublisher m_rackPositionPub;
  private final DoublePublisher m_rackVelocityPub;
  private final DoublePublisher m_rackVoltagePub;
  private final DoublePublisher m_rackSetpointPub;
  private final DoublePublisher m_rollerVelocityPub;
  private final DoublePublisher m_rollerVoltagePub;
  private final DoublePublisher m_rollerVoltageSetpointPub;
  private final DoublePublisher m_rackStatorCurrentPub;
  private final DoublePublisher m_rackSupplyCurrentPub;
  private final DoublePublisher m_rollerStatorCurrentPub;
  private final DoublePublisher m_rollerSupplyCurrentPub;
  private final BooleanPublisher m_atPositionPub;
  private final StringPublisher m_currentCommandPub;

  // Control requests
  private final MotionMagicTorqueCurrentFOC m_rackPositionRequest =
      new MotionMagicTorqueCurrentFOC(0);
  private final VoltageOut m_rollerVoltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  // Track setpoints for logging
  private double m_rackPositionSetpoint;
  private double m_rollerVoltageSetpoint;

  // State variable for feedingWiggleRackCommand oscillation direction
  private boolean m_wiggleGoingIn;

  // Tunable gains for rack
  private final DoubleSubscriber m_rackKSSub;
  private final DoubleSubscriber m_rackKGSub;
  private final DoubleSubscriber m_rackKVSub;
  private final DoubleSubscriber m_rackKASub;
  private final DoubleSubscriber m_rackKPSub;
  private final DoubleSubscriber m_rackKISub;
  private final DoubleSubscriber m_rackKDSub;
  private final DoublePublisher m_rackKSPub;
  private final DoublePublisher m_rackKGPub;
  private final DoublePublisher m_rackKVPub;
  private final DoublePublisher m_rackKAPub;
  private final DoublePublisher m_rackKPPub;
  private final DoublePublisher m_rackKIPub;
  private final DoublePublisher m_rackKDPub;

  private final Timer m_wiggleTimer;

  // Track last known gain values to detect changes
  private double m_lastRackKS;
  private double m_lastRackKG;
  private double m_lastRackKV;
  private double m_lastRackKA;
  private double m_lastRackKP;
  private double m_lastRackKI;
  private double m_lastRackKD;

  // Tuning mode NetworkTables controls
  private final BooleanSubscriber m_tuningEnabledSub;
  private final BooleanPublisher m_tuningEnabledPub;
  private final Trigger m_tuningEnabledTrigger;
  private final DoubleSubscriber m_tuningRackPositionSub;
  private final DoublePublisher m_tuningRackPositionPub;

  /** Creates a new Intake subsystem. */
  public Intake() {
    m_rackMotor = new TalonFX(IntakeConstants.RACK_MOTOR_ID);
    m_rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);

    configureRackMotor();
    configureRollerMotor();

    // Get rack status signals
    m_rackPosition = m_rackMotor.getPosition();
    m_rackVelocity = m_rackMotor.getVelocity();
    m_rackVoltage = m_rackMotor.getMotorVoltage();

    // Get roller status signals
    m_rollerVelocity = m_rollerMotor.getVelocity();
    m_rollerVoltage = m_rollerMotor.getMotorVoltage();

    // Get current status signals
    m_rackStatorCurrent = m_rackMotor.getStatorCurrent();
    m_rackSupplyCurrent = m_rackMotor.getSupplyCurrent();
    m_rollerStatorCurrent = m_rollerMotor.getStatorCurrent();
    m_rollerSupplyCurrent = m_rollerMotor.getSupplyCurrent();

    m_wiggleTimer = new Timer();

    // Set update frequencies
    StatusCode setUpdateFreqResult =
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            m_rackPosition,
            m_rackVelocity,
            m_rackVoltage,
            m_rollerVelocity,
            m_rollerVoltage,
            m_rackStatorCurrent,
            m_rackSupplyCurrent,
            m_rollerStatorCurrent,
            m_rollerSupplyCurrent);
    if (!setUpdateFreqResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply update frequency for intake subsystem!");
    }
    StatusCode optiResult = ParentDevice.optimizeBusUtilizationForAll(m_rackMotor, m_rollerMotor);
    if (!optiResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply optimization for intake subsystem!");
    }

    // Initialize NetworkTables publishers for logging
    NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Intake");
    m_rackPositionPub = intakeTable.getDoubleTopic("RackPositionRotations").publish();
    m_rackVelocityPub = intakeTable.getDoubleTopic("RackVelocityRPM").publish();
    m_rackVoltagePub = intakeTable.getDoubleTopic("RackVoltage").publish();
    m_rackSetpointPub = intakeTable.getDoubleTopic("RackSetpoint").publish();
    m_rollerVelocityPub = intakeTable.getDoubleTopic("RollerVelocityRPM").publish();
    m_rollerVoltagePub = intakeTable.getDoubleTopic("RollerVoltage").publish();
    m_rollerVoltageSetpointPub = intakeTable.getDoubleTopic("RollerVoltageSetpoint").publish();
    m_rackStatorCurrentPub = intakeTable.getDoubleTopic("RackStatorCurrent").publish();
    m_rackSupplyCurrentPub = intakeTable.getDoubleTopic("RackSupplyCurrent").publish();
    m_rollerStatorCurrentPub = intakeTable.getDoubleTopic("RollerStatorCurrent").publish();
    m_rollerSupplyCurrentPub = intakeTable.getDoubleTopic("RollerSupplyCurrent").publish();
    m_atPositionPub = intakeTable.getBooleanTopic("AtPosition").publish();
    m_currentCommandPub = intakeTable.getStringTopic("CurrentCommand").publish();

    // Initialize tunable gains for rack
    NetworkTable tuningTable = intakeTable.getSubTable("Tuning").getSubTable("RackGains");

    m_rackKSPub = tuningTable.getDoubleTopic("kS").publish();
    m_rackKSSub = tuningTable.getDoubleTopic("kS").subscribe(IntakeConstants.kRackKS);
    m_rackKSPub.set(IntakeConstants.kRackKS);
    m_lastRackKS = IntakeConstants.kRackKS;

    m_rackKGPub = tuningTable.getDoubleTopic("kG").publish();
    m_rackKGSub = tuningTable.getDoubleTopic("kG").subscribe(IntakeConstants.kRackKG);
    m_rackKGPub.set(IntakeConstants.kRackKG);
    m_lastRackKG = IntakeConstants.kRackKG;

    m_rackKVPub = tuningTable.getDoubleTopic("kV").publish();
    m_rackKVSub = tuningTable.getDoubleTopic("kV").subscribe(IntakeConstants.kRackKV);
    m_rackKVPub.set(IntakeConstants.kRackKV);
    m_lastRackKV = IntakeConstants.kRackKV;

    m_rackKAPub = tuningTable.getDoubleTopic("kA").publish();
    m_rackKASub = tuningTable.getDoubleTopic("kA").subscribe(IntakeConstants.kRackKA);
    m_rackKAPub.set(IntakeConstants.kRackKA);
    m_lastRackKA = IntakeConstants.kRackKA;

    m_rackKPPub = tuningTable.getDoubleTopic("kP").publish();
    m_rackKPSub = tuningTable.getDoubleTopic("kP").subscribe(IntakeConstants.kRackKP);
    m_rackKPPub.set(IntakeConstants.kRackKP);
    m_lastRackKP = IntakeConstants.kRackKP;

    m_rackKIPub = tuningTable.getDoubleTopic("kI").publish();
    m_rackKISub = tuningTable.getDoubleTopic("kI").subscribe(IntakeConstants.kRackKI);
    m_rackKIPub.set(IntakeConstants.kRackKI);
    m_lastRackKI = IntakeConstants.kRackKI;

    m_rackKDPub = tuningTable.getDoubleTopic("kD").publish();
    m_rackKDSub = tuningTable.getDoubleTopic("kD").subscribe(IntakeConstants.kRackKD);
    m_rackKDPub.set(IntakeConstants.kRackKD);
    m_lastRackKD = IntakeConstants.kRackKD;

    // Initialize tuning mode controls (in parent Tuning table)
    NetworkTable tuningParent = intakeTable.getSubTable("Tuning");
    m_tuningEnabledPub = tuningParent.getBooleanTopic("Enabled").publish();
    m_tuningEnabledSub = tuningParent.getBooleanTopic("Enabled").subscribe(false);
    m_tuningEnabledPub.set(false);
    m_tuningEnabledTrigger = new Trigger(m_tuningEnabledSub::get);
    m_tuningRackPositionPub = tuningParent.getDoubleTopic("RackPositionRotations").publish();
    m_tuningRackPositionSub =
        tuningParent
            .getDoubleTopic("RackPositionRotations")
            .subscribe(IntakeConstants.kRackStowedPosition);
    m_tuningRackPositionPub.set(IntakeConstants.kRackStowedPosition);

    // Initialize simulation
    m_rackSimState = m_rackMotor.getSimState();
    m_rollerSimState = m_rollerMotor.getSimState();

    // Rack and pinion simulation (linear motion) - using KrakenX44 to match Skip
    m_rackSim =
        new ElevatorSim(
            DCMotor.getKrakenX44Foc(1),
            IntakeConstants.RACK_GEAR_RATIO,
            IntakeConstants.RACK_CARRIAGE_MASS_KG,
            IntakeConstants.RACK_PINION_PITCH_RADIUS_METERS,
            IntakeConstants.RACK_MIN_EXTENSION_METERS,
            IntakeConstants.RACK_MAX_EXTENSION_METERS,
            false, // No gravity (horizontal rack)
            0.0); // Starting position (retracted)

    // Roller flywheel simulation - using KrakenX60 to match Skip
    m_rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                IntakeConstants.ROLLER_MOI,
                IntakeConstants.ROLLER_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));
  }

  private void configureRackMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor output configuration
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        RobotBase.isSimulation()
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    // Current limits - supply disabled to match Skip, torque current handles limiting
    config.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(false)
            .withSupplyCurrentLimitEnable(false);

    // Voltage limits (matching Skip)
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;

    // Feedback configuration - use gear ratio for mechanism position
    config.Feedback =
        new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.RACK_GEAR_RATIO);

    // Slot 0 - Position control (no gravity compensation for horizontal rack)
    config.Slot0 =
        new Slot0Configs()
            .withKS(IntakeConstants.kRackKS)
            .withKG(IntakeConstants.kRackKG)
            .withKV(IntakeConstants.kRackKV)
            .withKA(IntakeConstants.kRackKA)
            .withKP(IntakeConstants.kRackKP)
            .withKI(IntakeConstants.kRackKI)
            .withKD(IntakeConstants.kRackKD);

    // Motion Magic configuration
    config.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(IntakeConstants.kRackMotionMagicCruiseVelocity)
            .withMotionMagicAcceleration(IntakeConstants.kRackMotionMagicAcceleration);

    // Torque current limits for deployed hold mode
    config.TorqueCurrent =
        new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(IntakeConstants.RACK_STATOR_LIMIT)
            .withPeakReverseTorqueCurrent(-IntakeConstants.RACK_STATOR_LIMIT);

    StatusCode configResult = m_rackMotor.getConfigurator().apply(config);
    if (!configResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to intake rack motor!");
    }
  }

  private void configureRollerMotor() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(IntakeConstants.ROLLER_STATOR_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(IntakeConstants.ROLLER_SUPPLY_LIMIT));

    StatusCode configResult = m_rollerMotor.getConfigurator().apply(config);
    if (!configResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to intake roller motor!");
    }
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        m_rackPosition,
        m_rackVelocity,
        m_rackVoltage,
        m_rollerVelocity,
        m_rollerVoltage,
        m_rackStatorCurrent,
        m_rackSupplyCurrent,
        m_rollerStatorCurrent,
        m_rollerSupplyCurrent);

    // Publish signals to NetworkTables
    m_rackPositionPub.set(m_rackPosition.getValue().in(Rotations));
    m_rackVelocityPub.set(m_rackVelocity.getValue().in(RotationsPerSecond) * 60.0);
    m_rackVoltagePub.set(m_rackVoltage.getValue().in(Volts));
    m_rackSetpointPub.set(m_rackPositionSetpoint);
    m_rollerVelocityPub.set(m_rollerVelocity.getValue().in(RotationsPerSecond) * 60.0);
    m_rollerVoltagePub.set(m_rollerVoltage.getValue().in(Volts));
    m_rollerVoltageSetpointPub.set(m_rollerVoltageSetpoint);
    m_rackStatorCurrentPub.set(m_rackStatorCurrent.getValue().in(Amps));
    m_rackSupplyCurrentPub.set(m_rackSupplyCurrent.getValue().in(Amps));
    m_rollerStatorCurrentPub.set(m_rollerStatorCurrent.getValue().in(Amps));
    m_rollerSupplyCurrentPub.set(m_rollerSupplyCurrent.getValue().in(Amps));
    m_atPositionPub.set(atPosition());

    // Publish current command name
    Command currentCommand = getCurrentCommand();
    m_currentCommandPub.set(currentCommand != null ? currentCommand.getName() : "None");

    // Update mechanism pose for AdvantageScope 3D visualization
    // Convert pinion rotations to linear distance along the rack
    double pinionRotations = m_rackPosition.getValue().in(Rotations);
    double linearDistanceMeters =
        pinionRotations * 2.0 * Math.PI * IntakeConstants.RACK_PINION_PITCH_RADIUS_METERS;

    // Rack extends up and out (after 180° front/back swap)
    double rackAngleRad = Math.toRadians(IntakeConstants.RACK_ANGLE_DEGREES);

    // Only publish the delta from zeroed position - config.json handles base position
    double deltaX = -linearDistanceMeters * Math.cos(rackAngleRad);
    double deltaZ = -linearDistanceMeters * Math.sin(rackAngleRad);

    MechanismVisualizer.setPose(
        MechanismVisualizer.INTAKE_INDEX, new Pose3d(deltaX, 0, deltaZ, new Rotation3d()));

    // Check for tuning updates and apply if changed
    updateTunableGains();
  }

  /** Checks for tunable gain updates from NetworkTables and applies them to the rack motor. */
  private void updateTunableGains() {
    // Read current values from NetworkTables
    double kS = m_rackKSSub.get();
    double kG = m_rackKGSub.get();
    double kV = m_rackKVSub.get();
    double kA = m_rackKASub.get();
    double kP = m_rackKPSub.get();
    double kI = m_rackKISub.get();
    double kD = m_rackKDSub.get();

    // Check if gains have changed
    boolean gainsChanged =
        kS != m_lastRackKS
            || kG != m_lastRackKG
            || kV != m_lastRackKV
            || kA != m_lastRackKA
            || kP != m_lastRackKP
            || kI != m_lastRackKI
            || kD != m_lastRackKD;

    if (gainsChanged) {
      Slot0Configs slot0 =
          new Slot0Configs()
              .withKS(kS)
              .withKG(kG)
              .withKV(kV)
              .withKA(kA)
              .withKP(kP)
              .withKI(kI)
              .withKD(kD);

      m_rackMotor.getConfigurator().apply(slot0);

      // Update last known values
      m_lastRackKS = kS;
      m_lastRackKG = kG;
      m_lastRackKV = kV;
      m_lastRackKA = kA;
      m_lastRackKP = kP;
      m_lastRackKI = kI;
      m_lastRackKD = kD;
    }
  }

  @Override
  public void simulationPeriodic() {
    // Update supply voltage from battery
    m_rackSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_rollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update rack and pinion simulation
    m_rackSim.setInputVoltage(m_rackSimState.getMotorVoltageMeasure().in(Volts));
    m_rackSim.update(0.020);

    // Feed rack simulation results back to motor simulation
    // Convert linear position (meters) to pinion rotations
    // Linear distance = pinion rotations × 2π × pitch_radius
    // So pinion rotations = linear distance / (2π × pitch_radius)
    double pinionRotations =
        m_rackSim.getPositionMeters()
            / (2.0 * Math.PI * IntakeConstants.RACK_PINION_PITCH_RADIUS_METERS);
    double pinionVelocityRps =
        m_rackSim.getVelocityMetersPerSecond()
            / (2.0 * Math.PI * IntakeConstants.RACK_PINION_PITCH_RADIUS_METERS);

    // Convert mechanism (pinion) rotations to motor rotations for position
    // Note: Skip does NOT multiply velocity by gear ratio
    m_rackSimState.setRawRotorPosition(pinionRotations * IntakeConstants.RACK_GEAR_RATIO);
    m_rackSimState.setRotorVelocity(pinionVelocityRps);

    // Update roller simulation
    m_rollerSim.setInputVoltage(m_rollerSimState.getMotorVoltageMeasure().in(Volts));
    m_rollerSim.update(0.020);

    // Feed roller simulation results back to motor simulation
    m_rollerSimState.setRawRotorPosition(
        m_rollerSim.getAngularPosition().times(IntakeConstants.ROLLER_GEAR_RATIO));
    m_rollerSimState.setRotorVelocity(
        m_rollerSim.getAngularVelocity().times(IntakeConstants.ROLLER_GEAR_RATIO));
  }

  /**
   * Checks if the rack is at the target position.
   *
   * @return true if at position within tolerance.
   */
  public boolean atPosition() {
    return Math.abs(m_rackPosition.getValue().in(Rotations) - m_rackPositionSetpoint)
        < IntakeConstants.kRackPositionToleranceRotations;
  }

  /**
   * Creates a command to deploy the intake (rack down and run rollers).
   *
   * @return A command that deploys the intake.
   */
  public Command deployCommand() {
    return this.run(
            () -> {
              m_rackPositionSetpoint = IntakeConstants.kRackDeployedPosition;
              m_rackMotor.setControl(
                  m_rackPositionRequest.withPosition(IntakeConstants.kRackDeployedPosition));
              m_rollerVoltageSetpoint = IntakeConstants.kIntakeVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(IntakeConstants.kIntakeVoltage));
            })
        .finallyDo(
            () -> {
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("DeployIntake");
  }

  /**
   * Creates a command to deploy the intake (rack down and run rollers).
   *
   * @return A command that deploys the intake.
   */
  public Command autoDeployCommand() {
    return this.run(
            () -> {
              m_rackPositionSetpoint = IntakeConstants.kRackDeployedPosition;
              m_rackMotor.setControl(
                  m_rackPositionRequest.withPosition(IntakeConstants.kRackDeployedPosition));
              m_rollerVoltageSetpoint = IntakeConstants.kIntakeVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(IntakeConstants.kIntakeVoltage));
            })
        .until(
            () -> {
              return atPosition();
            })
        .finallyDo(
            () -> {
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("AutoDeployIntake");
  }

  public Command deployOnly() {
    return this.runOnce(
            () -> {
              m_rackPositionSetpoint = IntakeConstants.kRackDeployedPosition;
              m_rackMotor.setControl(
                  m_rackPositionRequest.withPosition(IntakeConstants.kRackDeployedPosition));
            })
        .withName("DeployOnly");
  }

  /**
   * Creates a command to stow the intake (rack up and stop rollers).
   *
   * @return A command that stows the intake.
   */
  public Command stowCommand() {
    return this.runOnce(
            () -> {
              m_rollerVoltageSetpoint = IntakeConstants.kIntakeVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(IntakeConstants.kIntakeVoltage));
              m_rackPositionSetpoint = IntakeConstants.kRackStowedPosition;
            })
        .andThen(Commands.waitSeconds(0.25))
        .andThen(
            this.run(
                () -> {
                  m_rackMotor.setControl(
                      m_rackPositionRequest.withPosition(IntakeConstants.kRackStowedPosition));
                }))
        .until(() -> atPosition())
        .finallyDo(
            () -> {
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("Stow Command");
  }

  public Command runRollers() {
    return this.run(
            () -> {
              m_rollerVoltageSetpoint = IntakeConstants.kIntakeVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(IntakeConstants.kIntakeVoltage));
            })
        .finallyDo(
            () -> {
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("RunIntakeRollers");
  }

  public Command runAutoRollers() {
    return this.runOnce(
            () -> {
              m_rollerVoltageSetpoint = IntakeConstants.kIntakeVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(IntakeConstants.kIntakeVoltage));
            })
        .withName("RunIntakeRollers");
  }

  public Command runRollersReverse() {
    return this.run(
            () -> {
              m_rollerVoltageSetpoint = -IntakeConstants.kIntakeVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(-IntakeConstants.kIntakeVoltage));
            })
        .finallyDo(
            () -> {
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("RunIntakeRollersReverse");
  }

  /**
   * Creates a command to eject game pieces (run rollers in reverse).
   *
   * @return A command that ejects game pieces.
   */
  public Command ejectCommand() {
    return this.run(
            () -> {
              m_rollerVoltageSetpoint = IntakeConstants.kEjectVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(IntakeConstants.kEjectVoltage));
            })
        .finallyDo(
            () -> {
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("EjectIntake");
  }

  /**
   * Creates a command to set the rack to a specific position.
   *
   * @param positionRotations The target position in rotations.
   * @return A command that moves the rack.
   */
  public Command setRackPositionCommand(double positionRotations) {
    return this.run(
            () -> {
              m_rackPositionSetpoint = positionRotations;
              m_rackMotor.setControl(m_rackPositionRequest.withPosition(positionRotations));
            })
        .withName("SetRackPosition");
  }

  /**
   * Creates a command to run the roller at a specific voltage.
   *
   * @param voltage The voltage to run at.
   * @return A command that runs the roller.
   */
  public Command runRollerVoltageCommand(double voltage) {
    return this.run(
            () -> {
              m_rollerVoltageSetpoint = voltage;
              m_rollerMotor.setControl(m_rollerVoltageRequest.withOutput(voltage));
            })
        .finallyDo(
            () -> {
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("RunIntakeRoller");
  }

  /**
   * Creates a command that stops both motors.
   *
   * @return A command that stops the intake.
   */
  public Command stopCommand() {
    return this.runOnce(
            () -> {
              m_rackPositionSetpoint = m_rackPosition.getValue().in(Rotations);
              m_rackMotor.setControl(m_neutralRequest);
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("StopIntake");
  }

  /**
   * Returns a trigger that is true when tuning mode is enabled via NetworkTables.
   *
   * @return the tuning enabled trigger
   */
  public Trigger tuningEnabledTrigger() {
    return m_tuningEnabledTrigger;
  }

  /**
   * Returns true if the rollers are currently running (voltage setpoint is non-zero).
   *
   * @return true if rollers are running.
   */
  public boolean isRollersRunning() {
    return m_rollerVoltageSetpoint != 0.0;
  }

  /**
   * Returns true if the intake is deployed (rack is at or targeting the deployed position).
   *
   * @return true if deployed.
   */
  public boolean isDeployed() {
    return m_rackPositionSetpoint == IntakeConstants.kRackDeployedPosition;
  }

  /**
   * Returns a trigger that is true when the intake is actively intaking (rollers running and
   * deployed).
   *
   * @return the intaking trigger.
   */
  public Trigger intakingTrigger() {
    return new Trigger(() -> isRollersRunning() && isDeployed());
  }

  /**
   * Creates a command that continuously wiggles the rack between the deployed position and 20
   * degrees above it while feeding. Returns the rack to the deployed position when the command
   * ends.
   *
   * @return A command that wiggles the rack during feeding.
   */
  public Command feedingWiggleRackCommand() {
    return this.run(
            () -> {
              double targetPos =
                  m_wiggleGoingIn
                      ? IntakeConstants.kRackDeployedPosition
                          + IntakeConstants.kRackFeedingWiggleOffset
                      : IntakeConstants.kRackDeployedPosition;
              m_rackPositionSetpoint = targetPos;
              m_rackMotor.setControl(m_rackPositionRequest.withPosition(targetPos));
              m_rollerVoltageSetpoint = IntakeConstants.kIntakeVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(IntakeConstants.kIntakeVoltage));
              if (m_wiggleTimer.hasElapsed(0.60)) {
                m_wiggleGoingIn = !m_wiggleGoingIn;
                m_wiggleTimer.reset();
                m_wiggleTimer.start();
              }
            })
        .beforeStarting(
            () -> {
              m_wiggleGoingIn = false;
              m_wiggleTimer.reset();
              m_wiggleTimer.start();
            })
        .finallyDo(
            () -> {
              m_rackPositionSetpoint = IntakeConstants.kRackDeployedPosition;
              m_rackMotor.setControl(
                  m_rackPositionRequest.withPosition(IntakeConstants.kRackDeployedPosition));
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("FeedingWiggleRack");
  }

  /**
   * Creates a command for tuning mode. Moves the rack to the position from
   * Intake/Tuning/RackPositionRotations. Use with tuningEnabledTrigger() to schedule based on the
   * Enabled entry.
   *
   * @return A command that runs the intake in tuning mode.
   */
  public Command tuningCommand() {
    return this.run(
            () -> {
              double targetPosition = m_tuningRackPositionSub.get();
              m_rackPositionSetpoint = targetPosition;
              m_rackMotor.setControl(m_rackPositionRequest.withPosition(targetPosition));
            })
        .finallyDo(() -> m_rackMotor.setControl(m_neutralRequest))
        .withName("IntakeTuning");
  }
}
