// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoublePublisher;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {

  private final TalonFX m_spindexerMotor;

  private final StatusSignal<AngularVelocity> m_motorVelocity;
  private final StatusSignal<Angle> m_motorPosition;
  private final StatusSignal<Voltage> m_motorVoltage;
  private final StatusSignal<Current> m_motorStatorCurrent;
  private final StatusSignal<Current> m_motorSupplyCurrent;

  // Simulation objects
  private final TalonFXSimState m_motorSimState;
  private final DCMotorSim m_spindexerSim;

  // NetworkTables publishers for logging
  private final DoublePublisher m_velocityPub;
  private final DoublePublisher m_voltagePub;
  private final DoublePublisher m_voltageSetpointPub;
  private final DoublePublisher m_statorCurrentPub;
  private final DoublePublisher m_supplyCurrentPub;
  private final StringPublisher m_currentCommandPub;

  // Control requests
  private final VoltageOut m_voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  // Track current voltage setpoint for logging
  private double m_currentVoltageSetpoint;

  /** Creates a new Spindexer. */
  public Spindexer() {
    m_spindexerMotor = new TalonFX(SpindexerConstants.SPINDEXER_MOTOR_ID);

    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        SpindexerConstants.INVERTED
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(SpindexerConstants.SPINDEXER_STATOR_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(SpindexerConstants.SPINDEXER_SUPPLY_LIMIT));

    StatusCode configResult = m_spindexerMotor.getConfigurator().apply(config);
    if (!configResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to spindexer motor!");
    }

    m_motorVelocity = m_spindexerMotor.getVelocity();
    m_motorPosition = m_spindexerMotor.getPosition();
    m_motorVoltage = m_spindexerMotor.getMotorVoltage();
    m_motorStatorCurrent = m_spindexerMotor.getStatorCurrent();
    m_motorSupplyCurrent = m_spindexerMotor.getSupplyCurrent();

    StatusCode setUpdateFreqResult =
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            m_motorVelocity,
            m_motorPosition,
            m_motorVoltage,
            m_motorStatorCurrent,
            m_motorSupplyCurrent);
    if (!setUpdateFreqResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply update frequency for spindexer subsystem!");
    }
    StatusCode optiResult = ParentDevice.optimizeBusUtilizationForAll(m_spindexerMotor);
    if (!optiResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply optimization for spindexer subsystem!");
    }

    // Initialize NetworkTables publishers for logging
    NetworkTable spindexerTable = NetworkTableInstance.getDefault().getTable("Spindexer");
    m_velocityPub = spindexerTable.getDoubleTopic("VelocityRPM").publish();
    m_voltagePub = spindexerTable.getDoubleTopic("Voltage").publish();
    m_voltageSetpointPub = spindexerTable.getDoubleTopic("VoltageSetpoint").publish();
    m_statorCurrentPub = spindexerTable.getDoubleTopic("StatorCurrent").publish();
    m_supplyCurrentPub = spindexerTable.getDoubleTopic("SupplyCurrent").publish();
    m_currentCommandPub = spindexerTable.getStringTopic("CurrentCommand").publish();

    // Initialize simulation
    m_motorSimState = m_spindexerMotor.getSimState();

    // Spindexer flywheel sim (powered by single Falcon 500)
    m_spindexerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getFalcon500Foc(1),
                SpindexerConstants.SPINDEXER_MOI,
                SpindexerConstants.SPINDEXER_GEAR_RATIO),
            DCMotor.getFalcon500Foc(1));
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        m_motorVelocity,
        m_motorPosition,
        m_motorVoltage,
        m_motorStatorCurrent,
        m_motorSupplyCurrent);

    // Publish signals to NetworkTables
    m_velocityPub.set(m_motorVelocity.getValue().in(RotationsPerSecond) * 60.0);
    m_voltagePub.set(m_motorVoltage.getValue().in(Volts));
    m_voltageSetpointPub.set(m_currentVoltageSetpoint);
    m_statorCurrentPub.set(m_motorStatorCurrent.getValue().in(Amps));
    m_supplyCurrentPub.set(m_motorSupplyCurrent.getValue().in(Amps));

    // Publish current command name
    Command currentCommand = getCurrentCommand();
    m_currentCommandPub.set(currentCommand != null ? currentCommand.getName() : "None");
  }

  @Override
  public void simulationPeriodic() {
    // Update supply voltage from battery
    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Get motor voltage and update physics simulation
    m_spindexerSim.setInputVoltage(m_motorSimState.getMotorVoltageMeasure().in(Volts));
    m_spindexerSim.update(0.020);

    // Update motor sim state with rotor position and velocity
    // Note: DCMotorSim returns mechanism values, multiply by gear ratio for rotor values
    m_motorSimState.setRawRotorPosition(
        m_spindexerSim.getAngularPosition().times(SpindexerConstants.SPINDEXER_GEAR_RATIO));
    m_motorSimState.setRotorVelocity(
        m_spindexerSim.getAngularVelocity().times(SpindexerConstants.SPINDEXER_GEAR_RATIO));
  }

  /**
   * Creates a command that runs the spindexer at the specified voltage.
   *
   * @param voltage The voltage to run at (positive = forward, negative = reverse)
   * @return A command that runs the spindexer
   */
  public Command runVoltageCommand(double voltage) {
    return this.run(
            () -> {
              m_currentVoltageSetpoint = voltage;
              m_spindexerMotor.setControl(m_voltageRequest.withOutput(voltage));
            })
        .withName("RunSpindexer");
  }

  /**
   * Creates a command that stops the spindexer motor.
   *
   * @return A command that stops the spindexer
   */
  public Command stopCommand() {
    return this.runOnce(
            () -> {
              m_currentVoltageSetpoint = 0.0;
              m_spindexerMotor.setControl(m_neutralRequest);
            })
        .withName("StopSpindexer");
  }

  /**
   * Creates a command that spins the spindexer at the constant voltage defined in constants. Stops
   * when the command ends.
   *
   * @return A command that spins the spindexer.
   */
  public Command spin() {
    return this.run(
            () -> {
              m_currentVoltageSetpoint = SpindexerConstants.kSpinVoltage;
              m_spindexerMotor.setControl(
                  m_voltageRequest.withOutput(SpindexerConstants.kSpinVoltage));
            })
        .finallyDo(
            () -> {
              m_currentVoltageSetpoint = 0.0;
              m_spindexerMotor.setControl(m_voltageRequest.withOutput(0));
            })
        .withName("Spin");
  }
  public Command spinReverse() {
    return this.run(
            () -> {
              m_currentVoltageSetpoint = -SpindexerConstants.kSpinVoltage;
              m_spindexerMotor.setControl(
                  m_voltageRequest.withOutput(-SpindexerConstants.kSpinVoltage));
            })
        .finallyDo(
            () -> {
              m_currentVoltageSetpoint = 0.0;
              m_spindexerMotor.setControl(m_voltageRequest.withOutput(0));
            })
        .withName("SpinReverse");
  }

  public Command stop() {
    return this.runOnce(
            () -> {
              m_currentVoltageSetpoint = 0.0;
              m_spindexerMotor.setControl(m_voltageRequest.withOutput(0));
            })
        .withName("Stop Spindexer");
  }
}
