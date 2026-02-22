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
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanPublisher;
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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.MechanismVisualizer;

/** Climber subsystem with a single motor for vertical climbing. */
public class Climber extends SubsystemBase {

  private final TalonFX m_motor;

  // Status signals
  private final StatusSignal<Angle> m_motorPosition;
  private final StatusSignal<AngularVelocity> m_motorVelocity;
  private final StatusSignal<Voltage> m_motorVoltage;
  private final StatusSignal<Current> m_statorCurrent;
  private final StatusSignal<Current> m_supplyCurrent;

  // Simulation objects
  private final TalonFXSimState m_motorSimState;
  private final ElevatorSim m_extendSim; // Light load for extending up
  private final ElevatorSim m_climbSim; // Heavy load for climbing (lifting robot)

  // Latched state - when true, we're hooked on the bar and lifting the robot
  private boolean m_isLatched;

  // NetworkTables publishers for logging
  private final DoublePublisher m_positionMetersPub;
  private final DoublePublisher m_positionRotationsPub;
  private final DoublePublisher m_velocityPub;
  private final DoublePublisher m_voltagePub;
  private final DoublePublisher m_setpointPub;
  private final DoublePublisher m_statorCurrentPub;
  private final DoublePublisher m_supplyCurrentPub;
  private final BooleanPublisher m_atPositionPub;
  private final BooleanPublisher m_isLatchedPub;
  private final StringPublisher m_currentCommandPub;

  // Control requests
  private final MotionMagicVoltage m_positionRequest =
      new MotionMagicVoltage(0).withEnableFOC(true);
  private final VoltageOut m_voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  // Track setpoint for logging (in meters)
  private double m_positionSetpoint;

  // Tunable gains
  private final DoubleSubscriber m_kSSub;
  private final DoubleSubscriber m_kGSub;
  private final DoubleSubscriber m_kVSub;
  private final DoubleSubscriber m_kASub;
  private final DoubleSubscriber m_kPSub;
  private final DoubleSubscriber m_kISub;
  private final DoubleSubscriber m_kDSub;
  private final DoublePublisher m_kSPub;
  private final DoublePublisher m_kGPub;
  private final DoublePublisher m_kVPub;
  private final DoublePublisher m_kAPub;
  private final DoublePublisher m_kPPub;
  private final DoublePublisher m_kIPub;
  private final DoublePublisher m_kDPub;

  // Track last known gain values to detect changes
  private double m_lastKS;
  private double m_lastKG;
  private double m_lastKV;
  private double m_lastKA;
  private double m_lastKP;
  private double m_lastKI;
  private double m_lastKD;

  /** Creates a new Climber subsystem. */
  public Climber() {
    m_motor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID);

    configureMotor();

    // Get status signals
    m_motorPosition = m_motor.getPosition();
    m_motorVelocity = m_motor.getVelocity();
    m_motorVoltage = m_motor.getMotorVoltage();
    m_statorCurrent = m_motor.getStatorCurrent();
    m_supplyCurrent = m_motor.getSupplyCurrent();

    // Set update frequencies
    StatusCode setUpdateFreqResult =
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            m_motorPosition,
            m_motorVelocity,
            m_motorVoltage,
            m_statorCurrent,
            m_supplyCurrent);
    if (!setUpdateFreqResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply update frequency for climber subsystem!");
    }
    StatusCode optiResult = ParentDevice.optimizeBusUtilizationForAll(m_motor);
    if (!optiResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply optimization for climber subsystem!");
    }

    // Initialize NetworkTables publishers for logging
    NetworkTable climberTable = NetworkTableInstance.getDefault().getTable("Climber");
    m_positionMetersPub = climberTable.getDoubleTopic("PositionMeters").publish();
    m_positionRotationsPub = climberTable.getDoubleTopic("PositionRotations").publish();
    m_velocityPub = climberTable.getDoubleTopic("VelocityMPS").publish();
    m_voltagePub = climberTable.getDoubleTopic("Voltage").publish();
    m_setpointPub = climberTable.getDoubleTopic("SetpointMeters").publish();
    m_statorCurrentPub = climberTable.getDoubleTopic("StatorCurrent").publish();
    m_supplyCurrentPub = climberTable.getDoubleTopic("SupplyCurrent").publish();
    m_atPositionPub = climberTable.getBooleanTopic("AtPosition").publish();
    m_isLatchedPub = climberTable.getBooleanTopic("IsLatched").publish();
    m_currentCommandPub = climberTable.getStringTopic("CurrentCommand").publish();

    // Initialize tunable gains
    NetworkTable tuningTable = climberTable.getSubTable("Tuning").getSubTable("Gains");

    m_kSPub = tuningTable.getDoubleTopic("kS").publish();
    m_kSSub = tuningTable.getDoubleTopic("kS").subscribe(ClimberConstants.kClimberKS);
    m_kSPub.set(ClimberConstants.kClimberKS);
    m_lastKS = ClimberConstants.kClimberKS;

    m_kGPub = tuningTable.getDoubleTopic("kG").publish();
    m_kGSub = tuningTable.getDoubleTopic("kG").subscribe(ClimberConstants.kClimberKG);
    m_kGPub.set(ClimberConstants.kClimberKG);
    m_lastKG = ClimberConstants.kClimberKG;

    m_kVPub = tuningTable.getDoubleTopic("kV").publish();
    m_kVSub = tuningTable.getDoubleTopic("kV").subscribe(ClimberConstants.kClimberKV);
    m_kVPub.set(ClimberConstants.kClimberKV);
    m_lastKV = ClimberConstants.kClimberKV;

    m_kAPub = tuningTable.getDoubleTopic("kA").publish();
    m_kASub = tuningTable.getDoubleTopic("kA").subscribe(ClimberConstants.kClimberKA);
    m_kAPub.set(ClimberConstants.kClimberKA);
    m_lastKA = ClimberConstants.kClimberKA;

    m_kPPub = tuningTable.getDoubleTopic("kP").publish();
    m_kPSub = tuningTable.getDoubleTopic("kP").subscribe(ClimberConstants.kClimberKP);
    m_kPPub.set(ClimberConstants.kClimberKP);
    m_lastKP = ClimberConstants.kClimberKP;

    m_kIPub = tuningTable.getDoubleTopic("kI").publish();
    m_kISub = tuningTable.getDoubleTopic("kI").subscribe(ClimberConstants.kClimberKI);
    m_kIPub.set(ClimberConstants.kClimberKI);
    m_lastKI = ClimberConstants.kClimberKI;

    m_kDPub = tuningTable.getDoubleTopic("kD").publish();
    m_kDSub = tuningTable.getDoubleTopic("kD").subscribe(ClimberConstants.kClimberKD);
    m_kDPub.set(ClimberConstants.kClimberKD);
    m_lastKD = ClimberConstants.kClimberKD;

    // Initialize simulation
    m_motorSimState = m_motor.getSimState();
    m_isLatched = false;

    // Light load simulation for extending up (just the climber hook/carriage)
    m_extendSim =
        new ElevatorSim(
            DCMotor.getFalcon500(1),
            ClimberConstants.CLIMBER_GEAR_RATIO,
            ClimberConstants.CLIMBER_CARRIAGE_MASS_KG,
            ClimberConstants.kDrumRadiusMeters,
            ClimberConstants.kMinHeightMeters,
            ClimberConstants.kMaxHeightMeters,
            true, // Simulate gravity
            0.0); // Starting position (retracted)

    // Heavy load simulation for climbing (lifting the robot)
    m_climbSim =
        new ElevatorSim(
            DCMotor.getFalcon500(1),
            ClimberConstants.CLIMBER_GEAR_RATIO,
            ClimberConstants.CLIMBER_ROBOT_MASS_KG,
            ClimberConstants.kDrumRadiusMeters,
            ClimberConstants.kMinHeightMeters,
            ClimberConstants.kMaxHeightMeters,
            true, // Simulate gravity
            0.0); // Starting position (retracted)
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor output configuration
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Current limits
    config.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(ClimberConstants.CLIMBER_STATOR_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(ClimberConstants.CLIMBER_SUPPLY_LIMIT);

    // Feedback configuration - convert rotations to linear distance using drum radius
    // Motor rotations -> mechanism rotations -> linear distance
    // SensorToMechanismRatio accounts for gear ratio
    // RotorToSensorRatio can be used for additional conversion
    config.Feedback =
        new FeedbackConfigs().withSensorToMechanismRatio(ClimberConstants.CLIMBER_GEAR_RATIO);

    // Slot 0 - Position control with gravity compensation
    config.Slot0 =
        new Slot0Configs()
            .withKS(ClimberConstants.kClimberKS)
            .withKG(ClimberConstants.kClimberKG)
            .withKV(ClimberConstants.kClimberKV)
            .withKA(ClimberConstants.kClimberKA)
            .withKP(ClimberConstants.kClimberKP)
            .withKI(ClimberConstants.kClimberKI)
            .withKD(ClimberConstants.kClimberKD)
            .withGravityType(GravityTypeValue.Elevator_Static);

    // Motion Magic configuration
    config.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ClimberConstants.kClimberMotionMagicCruiseVelocity)
            .withMotionMagicAcceleration(ClimberConstants.kClimberMotionMagicAcceleration);

    StatusCode configResult = m_motor.getConfigurator().apply(config);
    if (!configResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to climber motor!");
    }
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        m_motorPosition, m_motorVelocity, m_motorVoltage, m_statorCurrent, m_supplyCurrent);

    // Convert rotations to meters for logging
    double positionRotations = m_motorPosition.getValue().in(Rotations);
    double positionMeters = rotationsToMeters(positionRotations);
    double velocityRps = m_motorVelocity.getValue().in(RotationsPerSecond);
    double velocityMps = rotationsToMeters(velocityRps);

    // Publish signals to NetworkTables
    m_positionMetersPub.set(positionMeters);
    m_positionRotationsPub.set(positionRotations);
    m_velocityPub.set(velocityMps);
    m_voltagePub.set(m_motorVoltage.getValue().in(Volts));
    m_setpointPub.set(m_positionSetpoint);
    m_statorCurrentPub.set(m_statorCurrent.getValue().in(Amps));
    m_supplyCurrentPub.set(m_supplyCurrent.getValue().in(Amps));
    m_atPositionPub.set(atPosition());
    m_isLatchedPub.set(m_isLatched);

    // Publish current command name
    Command currentCommand = getCurrentCommand();
    m_currentCommandPub.set(currentCommand != null ? currentCommand.getName() : "None");

    // Update mechanism pose for AdvantageScope 3D visualization
    // Climber moves vertically, so we update the Z position
    MechanismVisualizer.setPose(
        MechanismVisualizer.CLIMBER_INDEX, new Pose3d(0, 0, positionMeters, new Rotation3d()));

    // Check for tuning updates and apply if changed
    updateTunableGains();
  }

  /** Checks for tunable gain updates from NetworkTables and applies them to the motor. */
  private void updateTunableGains() {
    // Read current values from NetworkTables
    double kS = m_kSSub.get();
    double kG = m_kGSub.get();
    double kV = m_kVSub.get();
    double kA = m_kASub.get();
    double kP = m_kPSub.get();
    double kI = m_kISub.get();
    double kD = m_kDSub.get();

    // Check if gains have changed
    boolean gainsChanged =
        kS != m_lastKS
            || kG != m_lastKG
            || kV != m_lastKV
            || kA != m_lastKA
            || kP != m_lastKP
            || kI != m_lastKI
            || kD != m_lastKD;

    if (gainsChanged) {
      Slot0Configs slot0 =
          new Slot0Configs()
              .withKS(kS)
              .withKG(kG)
              .withKV(kV)
              .withKA(kA)
              .withKP(kP)
              .withKI(kI)
              .withKD(kD)
              .withGravityType(GravityTypeValue.Elevator_Static);

      m_motor.getConfigurator().apply(slot0);

      // Update last known values
      m_lastKS = kS;
      m_lastKG = kG;
      m_lastKV = kV;
      m_lastKA = kA;
      m_lastKP = kP;
      m_lastKI = kI;
      m_lastKD = kD;
    }
  }

  @Override
  public void simulationPeriodic() {
    // Update supply voltage from battery
    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Select the appropriate simulation based on latched state
    // When latched, we're lifting the robot (heavy load)
    // When not latched, we're just extending the hook (light load)
    ElevatorSim activeSim = m_isLatched ? m_climbSim : m_extendSim;

    // Sync position between sims when switching (keep them in sync)
    double currentPosition = activeSim.getPositionMeters();
    if (m_isLatched) {
      m_extendSim.setState(currentPosition, activeSim.getVelocityMetersPerSecond());
    } else {
      m_climbSim.setState(currentPosition, activeSim.getVelocityMetersPerSecond());
    }

    // Update elevator simulation
    activeSim.setInputVoltage(m_motorSimState.getMotorVoltageMeasure().in(Volts));
    activeSim.update(0.020);

    // Feed simulation results back to motor simulation
    // Convert linear position to motor rotations
    double mechanismRotations = metersToRotations(activeSim.getPositionMeters());
    m_motorSimState.setRawRotorPosition(mechanismRotations * ClimberConstants.CLIMBER_GEAR_RATIO);
    m_motorSimState.setRotorVelocity(
        metersToRotations(activeSim.getVelocityMetersPerSecond())
            * ClimberConstants.CLIMBER_GEAR_RATIO);
  }

  /**
   * Converts mechanism rotations to linear meters.
   *
   * @param rotations The number of rotations.
   * @return The linear distance in meters.
   */
  private double rotationsToMeters(double rotations) {
    return rotations * 2.0 * Math.PI * ClimberConstants.kDrumRadiusMeters;
  }

  /**
   * Converts linear meters to mechanism rotations.
   *
   * @param meters The linear distance in meters.
   * @return The number of rotations.
   */
  private double metersToRotations(double meters) {
    return meters / (2.0 * Math.PI * ClimberConstants.kDrumRadiusMeters);
  }

  /**
   * Checks if the climber is at the target position.
   *
   * @return true if at position within tolerance.
   */
  public boolean atPosition() {
    double currentPosition = rotationsToMeters(m_motorPosition.getValue().in(Rotations));
    return Math.abs(currentPosition - m_positionSetpoint)
        < ClimberConstants.kPositionToleranceMeters;
  }

  /**
   * Creates a command to extend the climber to full height.
   *
   * @return A command that extends the climber.
   */
  public Command extendCommand() {
    return this.run(
            () -> {
              m_positionSetpoint = ClimberConstants.kExtendedPosition;
              double targetRotations = metersToRotations(ClimberConstants.kExtendedPosition);
              m_motor.setControl(m_positionRequest.withPosition(targetRotations));
            })
        .withName("ExtendClimber");
  }

  /**
   * Creates a command to retract the climber to minimum height.
   *
   * @return A command that retracts the climber.
   */
  public Command retractCommand() {
    return this.run(
            () -> {
              m_positionSetpoint = ClimberConstants.kRetractedPosition;
              double targetRotations = metersToRotations(ClimberConstants.kRetractedPosition);
              m_motor.setControl(m_positionRequest.withPosition(targetRotations));
            })
        .withName("RetractClimber");
  }

  /**
   * Creates a command to climb at a specific voltage (manual control).
   *
   * @param voltage The voltage to apply (positive = extend, negative = retract).
   * @return A command that runs the climber at the specified voltage.
   */
  public Command climbVoltageCommand(double voltage) {
    return this.run(
            () -> {
              m_motor.setControl(m_voltageRequest.withOutput(voltage));
            })
        .finallyDo(() -> m_motor.setControl(m_neutralRequest))
        .withName("ClimbVoltage");
  }

  /**
   * Creates a command to climb up at full voltage (manual control).
   *
   * @return A command that extends the climber at full voltage.
   */
  public Command climbUpCommand() {
    return climbVoltageCommand(ClimberConstants.kClimbVoltage).withName("ClimbUp");
  }

  /**
   * Creates a command to climb down at full voltage (manual control).
   *
   * @return A command that retracts the climber at full voltage.
   */
  public Command climbDownCommand() {
    return climbVoltageCommand(ClimberConstants.kRetractVoltage).withName("ClimbDown");
  }

  /**
   * Creates a command to move the climber to a specific position.
   *
   * @param positionMeters The target position in meters.
   * @return A command that moves the climber.
   */
  public Command setPositionCommand(double positionMeters) {
    return this.run(
            () -> {
              m_positionSetpoint = positionMeters;
              double targetRotations = metersToRotations(positionMeters);
              m_motor.setControl(m_positionRequest.withPosition(targetRotations));
            })
        .withName("SetClimberPosition");
  }

  /**
   * Creates a command that stops the climber motor.
   *
   * @return A command that stops the climber.
   */
  public Command stopCommand() {
    return this.runOnce(
            () -> {
              m_positionSetpoint = rotationsToMeters(m_motorPosition.getValue().in(Rotations));
              m_motor.setControl(m_neutralRequest);
            })
        .withName("StopClimber");
  }

  /**
   * Sets the latched state. When latched, simulation uses heavy load (robot weight). When not
   * latched, simulation uses light load (just the climber hook).
   *
   * @param latched true if the climber is hooked onto the bar.
   */
  public void setLatched(boolean latched) {
    m_isLatched = latched;
  }

  /**
   * Returns whether the climber is currently latched (hooked onto bar).
   *
   * @return true if latched.
   */
  public boolean isLatched() {
    return m_isLatched;
  }

  /**
   * Returns whether the climber is currently extended (past halfway point).
   *
   * @return true if extended.
   */
  public boolean isExtended() {
    double currentPosition = rotationsToMeters(m_motorPosition.getValue().in(Rotations));
    double midpoint = (ClimberConstants.kExtendedPosition + ClimberConstants.kRetractedPosition) / 2;
    return currentPosition > midpoint;
  }

  /**
   * Creates a command that sets the latched state to true. Call this when the climber hooks onto
   * the climbing bar.
   *
   * @return A command that latches the climber.
   */
  public Command latchCommand() {
    return this.runOnce(() -> m_isLatched = true).withName("LatchClimber");
  }

  /**
   * Creates a command that sets the latched state to false. Call this when the climber releases
   * from the bar.
   *
   * @return A command that unlatches the climber.
   */
  public Command unlatchCommand() {
    return this.runOnce(() -> m_isLatched = false).withName("UnlatchClimber");
  }

  /**
   * Creates a command to climb (retract while latched). This sets latched state and retracts.
   *
   * @return A command that performs the climb.
   */
  public Command climbCommand() {
    return latchCommand().andThen(retractCommand()).withName("Climb");
  }
}
