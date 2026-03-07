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
import frc.robot.Constants.KickerConstants;

public class Kicker extends SubsystemBase {

  private final TalonFX m_kickerMotor;

  private final StatusSignal<AngularVelocity> m_motorVelocity;
  private final StatusSignal<Angle> m_motorPosition;
  private final StatusSignal<Voltage> m_motorVoltage;
  private final StatusSignal<Current> m_motorStatorCurrent;
  private final StatusSignal<Current> m_motorSupplyCurrent;

  // Simulation objects
  private final TalonFXSimState m_motorSimState;
  private final DCMotorSim m_kickerSim;

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

  /** Creates a new thingyyy. */
  public Kicker() {
    m_kickerMotor = new TalonFX(KickerConstants.KICKER_MOTOR_ID);

    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(KickerConstants.KICKER_STATOR_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(KickerConstants.KICKER_SUPPLY_LIMIT));

    StatusCode configResult = m_kickerMotor.getConfigurator().apply(config);
    if (!configResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to kicker motor!");
    }

    m_motorVelocity = m_kickerMotor.getVelocity();
    m_motorPosition = m_kickerMotor.getPosition();
    m_motorVoltage = m_kickerMotor.getMotorVoltage();
    m_motorStatorCurrent = m_kickerMotor.getStatorCurrent();
    m_motorSupplyCurrent = m_kickerMotor.getSupplyCurrent();

    StatusCode setUpdateFreqResult =
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            m_motorVelocity,
            m_motorPosition,
            m_motorVoltage,
            m_motorStatorCurrent,
            m_motorSupplyCurrent);
    if (!setUpdateFreqResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply update frequency for kicker subsystem!");
    }
    StatusCode optiResult = ParentDevice.optimizeBusUtilizationForAll(m_kickerMotor);
    if (!optiResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply optimization for kicker subsystem!");
    }

    // Initialize NetworkTables publishers for logging
    NetworkTable kickerTable = NetworkTableInstance.getDefault().getTable("Kicker");
    m_velocityPub = kickerTable.getDoubleTopic("VelocityRPM").publish();
    m_voltagePub = kickerTable.getDoubleTopic("Voltage").publish();
    m_voltageSetpointPub = kickerTable.getDoubleTopic("VoltageSetpoint").publish();
    m_statorCurrentPub = kickerTable.getDoubleTopic("StatorCurrent").publish();
    m_supplyCurrentPub = kickerTable.getDoubleTopic("SupplyCurrent").publish();
    m_currentCommandPub = kickerTable.getStringTopic("CurrentCommand").publish();

    // Initialize simulation
    m_motorSimState = m_kickerMotor.getSimState();

    // Kicker flywheel sim (powered by single Falcon 500)
    m_kickerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getFalcon500Foc(1),
                KickerConstants.KICKER_MOI,
                KickerConstants.KICKER_GEAR_RATIO),
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
    m_kickerSim.setInputVoltage(m_motorSimState.getMotorVoltageMeasure().in(Volts));
    m_kickerSim.update(0.020);

    // Update motor sim state with rotor position and velocity
    // Note: DCMotorSim returns mechanism values, multiply by gear ratio for rotor values
    m_motorSimState.setRawRotorPosition(
        m_kickerSim.getAngularPosition().times(KickerConstants.KICKER_GEAR_RATIO));
    m_motorSimState.setRotorVelocity(
        m_kickerSim.getAngularVelocity().times(KickerConstants.KICKER_GEAR_RATIO));
  }

  /**
   * Creates a command that runs the Kicker at the specified voltage.
   *
   * @param voltage The voltage to run at (positive = forward, negative = reverse)
   * @return A command that runs the Kicker
   */
  public Command runVoltageCommand(double voltage) {
    return this.run(
            () -> {
              m_currentVoltageSetpoint = voltage;
              m_kickerMotor.setControl(m_voltageRequest.withOutput(voltage));
            })
        .withName("RunKicker");
  }

  /**
   * Creates a command that stops the Kicker motor.
   *
   * @return A command that stops the Kicker
   */
  public Command stopCommand() {
    return this.runOnce(
            () -> {
              m_currentVoltageSetpoint = 0.0;
              m_kickerMotor.setControl(m_neutralRequest);
            })
        .withName("StopKicker");
  }

  /**
   * Creates a command that spins the Kicker at the constant voltage defined in constants. Stops
   * when the command ends.
   *
   * @return A command that spins the Kicker.
   */
  public Command spin() {
    return this.run(
            () -> {
              m_currentVoltageSetpoint = KickerConstants.kSpinVoltage;
              m_kickerMotor.setControl(m_voltageRequest.withOutput(KickerConstants.kSpinVoltage));
            })
        .finallyDo(
            () -> {
              m_currentVoltageSetpoint = 0.0;
              m_kickerMotor.setControl(m_voltageRequest.withOutput(0));
            })
        .withName("SpinKicker");
  }

  public Command stop() {
    return this.runOnce(
            () -> {
              m_currentVoltageSetpoint = 0.0;
              m_kickerMotor.setControl(m_voltageRequest.withOutput(0));
            })
        .withName("Stop Kicker");
  }
}
