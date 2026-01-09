# FRC Robot 2026 - Context for Claude

This repository contains robot code for FRC Team 2053's 2026 season robot.

## Critical Code Standards

When writing or modifying Java code, you MUST follow these rules:

### Naming Conventions
- Member variables: `m_` prefix (e.g., `m_motor`, `m_speed`, `m_joystick`)
- Constants: `k` prefix (e.g., `kMaxSpeed`, `kMotorId`)
- Static final: `ALL_CAPS` (e.g., `MAX_VOLTAGE`)

### Imports
- **NEVER use wildcard imports** (no `import foo.*`)
- Use explicit imports only
- Static imports come first, then regular imports (alphabetical)

### Formatting
- Google Java Format style
- 100 character line limit
- 2-space indentation (spaces, not tabs)
- Always end Javadoc with a period

### Generated Files - DO NOT EDIT
- `src/main/java/frc/robot/generated/BuildConstants.java`
- `src/main/java/frc/robot/generated/TunerConstants.java`

These are auto-generated. Never modify them directly.

## Modern Command Based Practices
- Use subsystem level command factories, do not create command classes.
- Use Triggers where applicable for example:
```java
    new Trigger(this::atExtension)
        .negate()
        .debounce(CHECK_ZERO_SECONDS)
        .or(() -> !hasZeroed)
        .onTrue(Commands.runOnce(() -> notZeroedAlert.set(true)))
        .onFalse(Commands.runOnce(() -> notZeroedAlert.set(false)));
```

## Logging
- DO NOT USE SYSTEM.OUT.PRINTLN OR WRITE TO THE CONSOLE EVER.
- If you must debug or log commands, use Commands.Print()
- For logging state, we should make each subsystem have a networktable parent called the subsystem, with associated information under it.
- Log all input encoder and sensor data
- Log all output motor data
- Log all trigger state
- Example of logging a double below
```java
public class Example {
  // the publisher is an instance variable so its lifetime matches that of the class
  final DoublePublisher dblPub;
  public Example(DoubleTopic dblTopic) {
    // start publishing; the return value must be retained (in this case, via
    // an instance variable)
    dblPub = dblTopic.publish();
    // publish options may be specified using PubSubOption
    dblPub = dblTopic.publish(PubSubOption.keepDuplicates(true));
    // publishEx provides additional options such as setting initial
    // properties and using a custom type string. Using a custom type string for
    // types other than raw and string is not recommended. The properties string
    // must be a JSON map.
    dblPub = dblTopic.publishEx("double", "{\"myprop\": 5}");
  }
  public void periodic() {
    // publish a default value
    dblPub.setDefault(0.0);
    // publish a value with current timestamp
    dblPub.set(1.0);
    dblPub.set(2.0, 0);  // 0 = use current time
    // publish a value with a specific timestamp; NetworkTablesJNI.now() can
    // be used to get the current time. On the roboRIO, this is the same as
    // the FPGA timestamp (e.g. RobotController.getFPGATime())
    long time = NetworkTablesJNI.now();
    dblPub.set(3.0, time);
    // publishers also implement the appropriate Consumer functional interface;
    // this example assumes void myFunc(DoubleConsumer func) exists
    myFunc(dblPub);
  }
  // often not required in robot code, unless this class doesn't exist for
  // the lifetime of the entire robot program, in which case close() needs to be
  // called to stop publishing
  public void close() {
    // stop publishing
    dblPub.close();
  }
}
```

## Alerts
- Create Alerts to represent failures during system operation. For example, a motor not communicating,
other sensors not responding, motors going past limits, or the robot thinks its outside the field. Here is an example.
```java
class Robot {
  Alert alert = new Alert("Something went wrong", AlertType.kWarning);

  periodic() {
    alert.set(...);
  }
}
```

## Controlling motors
- We use phoenix 6 for all our motors.
- ALWAYS use onboard motor control (open loop, pid, motion profiling, etc).
- Here are examples of control requests and status signals to recieve data from the controller
```java
    // Control requests
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorCurrent;
```

- When setting up a motor for a user, you should ask what kind of motor it is, its gear ratio, and its can id, and how we are controlling it. Then create a configureMotors() function to encode this
```java
        motor = new TalonFX(canID);
        // Configure the motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Set neutral mode to brake (or Coast)
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Set current limits (required)
        config.CurrentLimits.SupplyCurrentLimit = 80; // Limit to 80 amps supply side
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        // Set stator current limits (required)
        config.CurrentLimits.StatorCurrentLimit = 60; // Limit to 60 amps supply side
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        // Invert motor if needed
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Change to Clockwise_Positive if motor runs backward
        // Apply configuration
        motor.getConfigurator().apply(config);
        // Get status signals
        motorPosition = motor.getPosition();
        motorVelocity = motor.getVelocity();
        motorVoltage = motor.getMotorVoltage();
        motorCurrent = motor.getSupplyCurrent();
```

## Project Structure

- `RobotContainer.java` - Robot configuration, subsystem initialization, button bindings
- `Robot.java` - Main robot class, lifecycle methods
- `Telemetry.java` - NetworkTables telemetry publishing for swerve state ONLY
- `subsystems/CommandSwerveDrivetrain.java` - Swerve drive subsystem (CTRE Phoenix 6)

## Technology Stack

- **WPILib 2026**: FRC framework
- **CTRE Phoenix 6**: Motor controllers, sensors
- **Command-based architecture**: Use `SubsystemBase` and `Command` patterns
- **Java 17**
- **Gradle build system**

## Code Quality

Before suggesting students commit code, remind them to run:
```bash
./gradlew javaFormat
```

This runs Spotless, Checkstyle, and PMD checks that CI will enforce.

## Common Patterns

### Creating a Subsystem
```java
public class ExampleSubsystem extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(CAN_ID);

  public ExampleSubsystem() {
    // Config here
  }

  public Command runMotor(double speed) {
    return run(() -> m_motor.set(speed)).withName("RunMotor");
  }
}
```

### Button Bindings (in RobotContainer)
```java
m_joystick.a().whileTrue(subsystem.command());
m_joystick.b().onTrue(subsystem.oneTimeCommand());
```

### CTRE Phoenix 6 Motors
```java
private final TalonFX m_motor = new TalonFX(CAN_ID);
m_motor.set(percentOutput); // -1.0 to 1.0
m_motor.setVoltage(volts);
```

## When Helping Students

1. Always follow the naming conventions above
2. Explain what the code does (they're learning)
3. Keep changes focused and simple
4. Remind them to test on the robot
5. Suggest running `./gradlew javaFormat` before committing

## Build Commands

- `./gradlew build` - Compile and run tests
- `./gradlew javaFormat` - Run all code quality checks
- `./gradlew spotlessApply` - Auto-fix formatting
- `./gradlew deploy` - Deploy to robot (student runs this, not you)
- `./gradlew simulateJava` - Run simulation
