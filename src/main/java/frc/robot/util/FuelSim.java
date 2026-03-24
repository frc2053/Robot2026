// https://github.com/hammerheads5000/FuelSim

package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class FuelSim {
  protected static final double PERIOD = 0.02; // sec
  protected static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81); // m/s^2
  // Room temperature dry air density: https://en.wikipedia.org/wiki/Density_of_air#Dry_air
  protected static final double AIR_DENSITY = 1.2041; // kg/m^3
  protected static final double FIELD_COR =
      Math.sqrt(22 / 51.5); // coefficient of restitution with the field
  protected static final double FUEL_COR = 0.5; // coefficient of restitution with another fuel
  protected static final double NET_COR = 0.2; // coefficient of restitution with the net
  protected static final double ROBOT_COR = 0.1; // coefficient of restitution with a robot
  protected static final double FUEL_RADIUS = 0.075;
  protected static final double FIELD_LENGTH = 16.51;
  protected static final double FIELD_WIDTH = 8.04;
  protected static final double TRENCH_WIDTH = 1.265;
  protected static final double TRENCH_BLOCK_WIDTH = 0.305;
  protected static final double TRENCH_HEIGHT = 0.565;
  protected static final double TRENCH_BAR_HEIGHT = 0.102;
  protected static final double TRENCH_BAR_WIDTH = 0.152;
  protected static final double FRICTION =
      0.1; // proportion of horizontal vel to lose per sec while on ground
  protected static final double FUEL_MASS = 0.448 * 0.45392; // kgs
  protected static final double FUEL_CROSS_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS;
  // Drag coefficient of smooth sphere:
  // https://en.wikipedia.org/wiki/Drag_coefficient#/media/File:14ilf1l.svg
  protected static final double DRAG_COF = 0.47; // dimensionless
  protected static final double DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA;

  protected static final Translation3d[] FIELD_XZ_LINE_STARTS = {
    new Translation3d(0, 0, 0),
    new Translation3d(3.96, 1.57, 0),
    new Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
    new Translation3d(4.61, 1.57, 0.165),
    new Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
    new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
    new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
    new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
    new Translation3d(3.96, TRENCH_WIDTH, TRENCH_HEIGHT),
    new Translation3d(3.96, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
    new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2,
        FIELD_WIDTH - 1.57,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
  };

  protected static final Translation3d[] FIELD_XZ_LINE_ENDS = {
    new Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0),
    new Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
    new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
    new Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
    new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0),
    new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(
        4.61 + TRENCH_BAR_WIDTH / 2,
        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
  };

  protected static class Fuel {
    protected Translation3d m_pos;
    protected Translation3d m_vel;

    protected Fuel(Translation3d pos, Translation3d vel) {
      this.m_pos = pos;
      this.m_vel = vel;
    }

    protected Fuel(Translation3d pos) {
      this(pos, new Translation3d());
    }

    protected void update(boolean simulateAirResistance, int subticks) {
      m_pos = m_pos.plus(m_vel.times(PERIOD / subticks));
      if (m_pos.getZ() > FUEL_RADIUS) {
        Translation3d fg = GRAVITY.times(FUEL_MASS);
        Translation3d fd = new Translation3d();

        double speed = m_vel.getNorm();
        if (simulateAirResistance && speed > 1e-6) {
          fd = m_vel.times(-DRAG_FORCE_FACTOR * speed);
        }

        Translation3d accel = fg.plus(fd).div(FUEL_MASS);
        m_vel = m_vel.plus(accel.times(PERIOD / subticks));
      }
      if (Math.abs(m_vel.getZ()) < 0.05 && m_pos.getZ() <= FUEL_RADIUS + 0.03) {
        m_vel = new Translation3d(m_vel.getX(), m_vel.getY(), 0);
        m_vel = m_vel.times(1 - FRICTION * PERIOD / subticks);
        // m_pos = new Translation3d(m_pos.getX(), m_pos.getY(), FUEL_RADIUS);
      }
      handleFieldCollisions(subticks);
    }

    protected void handleXZLineCollision(Translation3d lineStart, Translation3d lineEnd) {
      if (m_pos.getY() < lineStart.getY() || m_pos.getY() > lineEnd.getY()) {
        return; // not within y range
      }
      // Convert into 2D
      Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getZ());
      Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getZ());
      Translation2d pos2d = new Translation2d(m_pos.getX(), m_pos.getZ());
      Translation2d lineVec = end2d.minus(start2d);

      // Get closest point on line
      Translation2d projected =
          start2d.plus(lineVec.times(pos2d.minus(start2d).dot(lineVec) / lineVec.getSquaredNorm()));

      if (projected.getDistance(start2d) + projected.getDistance(end2d) > lineVec.getNorm()) {
        return; // projected point not on line
      }
      double dist = pos2d.getDistance(projected);
      if (dist > FUEL_RADIUS) {
        return; // not intersecting line
      }
      // Back into 3D
      Translation3d normal =
          new Translation3d(-lineVec.getY(), 0, lineVec.getX()).div(lineVec.getNorm());

      // Apply collision response
      m_pos = m_pos.plus(normal.times(FUEL_RADIUS - dist));
      if (m_vel.dot(normal) > 0) {
        return; // already moving away from line
      }
      m_vel = m_vel.minus(normal.times((1 + FIELD_COR) * m_vel.dot(normal)));
    }

    protected void handleFieldCollisions(int subticks) {
      // floor and bumps
      for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
        handleXZLineCollision(FIELD_XZ_LINE_STARTS[i], FIELD_XZ_LINE_ENDS[i]);
      }

      // edges
      if (m_pos.getX() < FUEL_RADIUS && m_vel.getX() < 0) {
        m_pos = m_pos.plus(new Translation3d(FUEL_RADIUS - m_pos.getX(), 0, 0));
        m_vel = m_vel.plus(new Translation3d(-(1 + FIELD_COR) * m_vel.getX(), 0, 0));
      } else if (m_pos.getX() > FIELD_LENGTH - FUEL_RADIUS && m_vel.getX() > 0) {
        m_pos = m_pos.plus(new Translation3d(FIELD_LENGTH - FUEL_RADIUS - m_pos.getX(), 0, 0));
        m_vel = m_vel.plus(new Translation3d(-(1 + FIELD_COR) * m_vel.getX(), 0, 0));
      }

      if (m_pos.getY() < FUEL_RADIUS && m_vel.getY() < 0) {
        m_pos = m_pos.plus(new Translation3d(0, FUEL_RADIUS - m_pos.getY(), 0));
        m_vel = m_vel.plus(new Translation3d(0, -(1 + FIELD_COR) * m_vel.getY(), 0));
      } else if (m_pos.getY() > FIELD_WIDTH - FUEL_RADIUS && m_vel.getY() > 0) {
        m_pos = m_pos.plus(new Translation3d(0, FIELD_WIDTH - FUEL_RADIUS - m_pos.getY(), 0));
        m_vel = m_vel.plus(new Translation3d(0, -(1 + FIELD_COR) * m_vel.getY(), 0));
      }

      // hubs
      handleHubCollisions(Hub.BLUE_HUB, subticks);
      handleHubCollisions(Hub.RED_HUB, subticks);

      handleTrenchCollisions();
    }

    protected void handleHubCollisions(Hub hub, int subticks) {
      hub.handleHubInteraction(this, subticks);
      hub.fuelCollideSide(this);

      double netCollision = hub.fuelHitNet(this);
      if (netCollision != 0) {
        m_pos = m_pos.plus(new Translation3d(netCollision, 0, 0));
        m_vel = new Translation3d(-m_vel.getX() * NET_COR, m_vel.getY() * NET_COR, m_vel.getZ());
      }
    }

    protected void handleTrenchCollisions() {
      fuelCollideRectangle(
          this,
          new Translation3d(3.96, TRENCH_WIDTH, 0),
          new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(3.96, FIELD_WIDTH - 1.57, 0),
          new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, 0),
          new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, 0),
          new Translation3d(
              FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
          new Translation3d(
              4.61 + TRENCH_BAR_WIDTH / 2,
              TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
              TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
          new Translation3d(
              4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
          new Translation3d(
              FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
              TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
              TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(
              FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
          new Translation3d(
              FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
              FIELD_WIDTH,
              TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
    }

    protected void addImpulse(Translation3d impulse) {
      m_vel = m_vel.plus(impulse);
    }
  }

  protected static void handleFuelCollision(Fuel a, Fuel b) {
    Translation3d normal = a.m_pos.minus(b.m_pos);
    double distance = normal.getNorm();
    if (distance == 0) {
      normal = new Translation3d(1, 0, 0);
      distance = 1;
    }
    normal = normal.div(distance);
    double impulse = 0.5 * (1 + FUEL_COR) * (b.m_vel.minus(a.m_vel).dot(normal));
    double intersection = FUEL_RADIUS * 2 - distance;
    a.m_pos = a.m_pos.plus(normal.times(intersection / 2));
    b.m_pos = b.m_pos.minus(normal.times(intersection / 2));
    a.addImpulse(normal.times(impulse));
    b.addImpulse(normal.times(-impulse));
  }

  protected static final double CELL_SIZE = 0.25;
  protected static final int GRID_COLS = (int) Math.ceil(FIELD_LENGTH / CELL_SIZE);
  protected static final int GRID_ROWS = (int) Math.ceil(FIELD_WIDTH / CELL_SIZE);

  @SuppressWarnings("unchecked")
  protected final ArrayList<Fuel>[][] m_grid = new ArrayList[GRID_COLS][GRID_ROWS];

  private final ArrayList<ArrayList<Fuel>> m_activeCells = new ArrayList<>();

  protected void handleFuelCollisions(ArrayList<Fuel> fuels) {
    // Clear grid
    for (ArrayList<Fuel> cell : m_activeCells) {
      cell.clear();
    }
    m_activeCells.clear();

    // Populate grid
    for (Fuel fuel : fuels) {
      int col = (int) (fuel.m_pos.getX() / CELL_SIZE);
      int row = (int) (fuel.m_pos.getY() / CELL_SIZE);

      if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
        m_grid[col][row].add(fuel);
        if (m_grid[col][row].size() == 1) {
          m_activeCells.add(m_grid[col][row]);
        }
      }
    }

    // Check collisions
    for (Fuel fuel : fuels) {
      int col = (int) (fuel.m_pos.getX() / CELL_SIZE);
      int row = (int) (fuel.m_pos.getY() / CELL_SIZE);

      // Check 3x3 neighbor cells
      for (int i = col - 1; i <= col + 1; i++) {
        for (int j = row - 1; j <= row + 1; j++) {
          if (i >= 0 && i < GRID_COLS && j >= 0 && j < GRID_ROWS) {
            for (Fuel other : m_grid[i][j]) {
              if (!fuel.equals(other)
                  && fuel.m_pos.getDistance(other.m_pos) < FUEL_RADIUS * 2
                  && fuel.hashCode() < other.hashCode()) {
                handleFuelCollision(fuel, other);
              }
            }
          }
        }
      }
    }
  }

  protected ArrayList<Fuel> m_fuels = new ArrayList<>();
  protected boolean m_running;
  protected boolean m_simulateAirResistance;
  protected Supplier<Pose2d> m_robotPoseSupplier;
  protected Supplier<ChassisSpeeds> m_robotFieldSpeedsSupplier;
  protected double m_robotWidth; // size along the robot's y axis
  protected double m_robotLength; // size along the robot's x axis
  protected double m_bumperHeight;
  protected ArrayList<SimIntake> m_intakes = new ArrayList<>();
  protected int m_subticks = 5;
  protected double m_loggingFreqHz = 10;
  protected Timer m_loggingTimer = new Timer();

  /**
   * Creates a new instance of FuelSim.
   *
   * @param tableKey NetworkTable to log fuel positions to as an array of {@link Translation3d}
   *     structs.
   */
  public FuelSim(String tableKey) {
    // Initialize grid
    for (int i = 0; i < GRID_COLS; i++) {
      for (int j = 0; j < GRID_ROWS; j++) {
        m_grid[i][j] = new ArrayList<Fuel>();
      }
    }

    m_fuelPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic(tableKey + "/Fuels", Translation3d.struct)
            .publish();
  }

  /** Creates a new instance of FuelSim with log path "/Fuel Simulation". */
  public FuelSim() {
    this("/Fuel Simulation");
  }

  /** Clears the field of fuel. */
  public void clearFuel() {
    m_fuels.clear();
  }

  /** Spawns fuel in the neutral zone and depots. */
  public void spawnStartingFuel() {
    // Center fuel
    Translation3d center = new Translation3d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, FUEL_RADIUS);
    for (int i = 0; i < 15; i++) {
      for (int j = 0; j < 6; j++) {
        m_fuels.add(
            new Fuel(
                center.plus(new Translation3d(0.076 + 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
        m_fuels.add(
            new Fuel(
                center.plus(new Translation3d(-0.076 - 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
        m_fuels.add(
            new Fuel(
                center.plus(new Translation3d(0.076 + 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
        m_fuels.add(
            new Fuel(
                center.plus(
                    new Translation3d(-0.076 - 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
      }
    }

    // Depots
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
        m_fuels.add(
            new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 + 0.076 + 0.152 * i, FUEL_RADIUS)));
        m_fuels.add(
            new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 - 0.076 - 0.152 * i, FUEL_RADIUS)));
        m_fuels.add(
            new Fuel(
                new Translation3d(
                    FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 + 0.076 + 0.152 * i, FUEL_RADIUS)));
        m_fuels.add(
            new Fuel(
                new Translation3d(
                    FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 - 0.076 - 0.152 * i, FUEL_RADIUS)));
      }
    }

    // DEBUG: Log XZ lines
    // Translation3d[][] lines = new Translation3d[FIELD_XZ_LINE_STARTS.length][2];
    // for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
    //     lines[i][0] = FIELD_XZ_LINE_STARTS[i];
    //     lines[i][1] = FIELD_XZ_LINE_ENDS[i];
    // }

    // Logger.recordOutput("Fuel Simulation/Lines (debug)", lines);
  }

  protected StructArrayPublisher<Translation3d> m_fuelPublisher;

  /** Adds array of Translation3d's to NetworkTables at tableKey + "/Fuels". */
  public void logFuels() {
    m_fuelPublisher.set(m_fuels.stream().map(fuel -> fuel.m_pos).toArray(Translation3d[]::new));
  }

  /** Starts the simulation. updateSim must still be called every loop. */
  public void start() {
    m_running = true;
    m_loggingTimer.restart();
  }

  /** Pauses the simulation. */
  public void stop() {
    m_running = false;
    m_loggingTimer.stop();
  }

  /** Enables accounting for drag force in physics step. */
  public void enableAirResistance() {
    m_simulateAirResistance = true;
  }

  /**
   * Sets the number of physics iterations per loop (0.02s).
   *
   * @param subticks physics iteration per loop (default: 5)
   */
  public void setSubticks(int subticks) {
    this.m_subticks = subticks;
  }

  /**
   * Sets the frequency to publish fuel translations to NetworkTables. Used to improve performance
   * in AdvantageScope.
   *
   * @param loggingFreqHz update frequency in hertz
   */
  public void setLoggingFrequency(double loggingFreqHz) {
    this.m_loggingFreqHz = loggingFreqHz;
  }

  /**
   * Registers a robot with the fuel simulator.
   *
   * @param width from left to right (y-axis)
   * @param length from front to back (x-axis)
   * @param bumperHeight the height of the bumpers
   * @param poseSupplier the pose supplier for the robot
   * @param fieldSpeedsSupplier field-relative ChassisSpeeds supplier
   */
  public void registerRobot(
      double width,
      double length,
      double bumperHeight,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this.m_robotPoseSupplier = poseSupplier;
    this.m_robotFieldSpeedsSupplier = fieldSpeedsSupplier;
    this.m_robotWidth = width;
    this.m_robotLength = length;
    this.m_bumperHeight = bumperHeight;
  }

  /**
   * Registers a robot with the fuel simulator.
   *
   * @param width from left to right (y-axis)
   * @param length from front to back (x-axis)
   * @param bumperHeight from the ground
   * @param poseSupplier the pose supplier for the robot
   * @param fieldSpeedsSupplier field-relative ChassisSpeeds supplier
   */
  public void registerRobot(
      Distance width,
      Distance length,
      Distance bumperHeight,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this.m_robotPoseSupplier = poseSupplier;
    this.m_robotFieldSpeedsSupplier = fieldSpeedsSupplier;
    this.m_robotWidth = width.in(Meters);
    this.m_robotLength = length.in(Meters);
    this.m_bumperHeight = bumperHeight.in(Meters);
  }

  /** To be called periodically. Will do nothing if sim is not running. */
  public void updateSim() {
    if (!m_running) {
      return;
    }

    stepSim();
  }

  /** Runs the simulation forward 1 time step (0.02s). */
  public void stepSim() {
    for (int i = 0; i < m_subticks; i++) {
      for (Fuel fuel : m_fuels) {
        fuel.update(this.m_simulateAirResistance, this.m_subticks);
      }

      handleFuelCollisions(m_fuels);

      if (m_robotPoseSupplier != null) {
        handleRobotCollisions(m_fuels);
        handleIntakes(m_fuels);
      }
    }

    if (m_loggingTimer.advanceIfElapsed(1.0 / m_loggingFreqHz)) {
      logFuels();
    }
  }

  /**
   * Adds a fuel onto the field.
   *
   * @param pos Position to spawn at
   * @param vel Initial velocity vector
   */
  public void spawnFuel(Translation3d pos, Translation3d vel) {
    m_fuels.add(new Fuel(pos, vel));
  }

  /**
   * Spawns a fuel onto the field with a specified launch velocity and angles, accounting for robot
   * movement.
   *
   * @param launchVelocity Initial launch velocity
   * @param hoodAngle Hood angle where 0 is launching horizontally and 90 degrees is launching
   *     straight up
   * @param turretYaw <i>Robot-relative</i> turret yaw
   * @param launchHeight Height of the fuel to launch at. Make sure this is higher than your robot's
   *     bumper height, or else it will collide with your robot immediately.
   * @throws IllegalStateException if robot is not registered
   */
  public void launchFuel(
      LinearVelocity launchVelocity, Angle hoodAngle, Angle turretYaw, Distance launchHeight) {
    if (m_robotPoseSupplier == null || m_robotFieldSpeedsSupplier == null) {
      throw new IllegalStateException("Robot must be registered before launching fuel.");
    }

    Pose3d launchPose =
        new Pose3d(this.m_robotPoseSupplier.get())
            .plus(
                new Transform3d(
                    new Translation3d(Meters.zero(), Meters.zero(), launchHeight),
                    Rotation3d.kZero));
    ChassisSpeeds fieldSpeeds = this.m_robotFieldSpeedsSupplier.get();

    double horizontalVel = Math.cos(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
    double verticalVel = Math.sin(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
    double xVel =
        horizontalVel
            * Math.cos(turretYaw.plus(launchPose.getRotation().getMeasureZ()).in(Radians));
    double yVel =
        horizontalVel
            * Math.sin(turretYaw.plus(launchPose.getRotation().getMeasureZ()).in(Radians));

    xVel += fieldSpeeds.vxMetersPerSecond;
    yVel += fieldSpeeds.vyMetersPerSecond;

    spawnFuel(launchPose.getTranslation(), new Translation3d(xVel, yVel, verticalVel));
  }

  protected void handleRobotCollision(Fuel fuel, Pose2d robot, Translation2d robotVel) {
    Translation2d relativePos =
        new Pose2d(fuel.m_pos.toTranslation2d(), Rotation2d.kZero)
            .relativeTo(robot)
            .getTranslation();

    if (fuel.m_pos.getZ() > m_bumperHeight) {
      return; // above bumpers
    }
    double distanceToBottom = -FUEL_RADIUS - m_robotLength / 2 - relativePos.getX();
    double distanceToTop = -FUEL_RADIUS - m_robotLength / 2 + relativePos.getX();
    double distanceToRight = -FUEL_RADIUS - m_robotWidth / 2 - relativePos.getY();
    double distanceToLeft = -FUEL_RADIUS - m_robotWidth / 2 + relativePos.getY();

    // not inside robot
    if (distanceToBottom > 0 || distanceToTop > 0 || distanceToRight > 0 || distanceToLeft > 0) {
      return;
    }

    Translation2d posOffset;
    // find minimum distance to side and send corresponding collision response
    if (distanceToBottom >= distanceToTop
        && distanceToBottom >= distanceToRight
        && distanceToBottom >= distanceToLeft) {
      posOffset = new Translation2d(distanceToBottom, 0);
    } else if (distanceToTop >= distanceToBottom
        && distanceToTop >= distanceToRight
        && distanceToTop >= distanceToLeft) {
      posOffset = new Translation2d(-distanceToTop, 0);
    } else if (distanceToRight >= distanceToBottom
        && distanceToRight >= distanceToTop
        && distanceToRight >= distanceToLeft) {
      posOffset = new Translation2d(0, distanceToRight);
    } else {
      posOffset = new Translation2d(0, -distanceToLeft);
    }

    posOffset = posOffset.rotateBy(robot.getRotation());
    fuel.m_pos = fuel.m_pos.plus(new Translation3d(posOffset));
    Translation2d normal = posOffset.div(posOffset.getNorm());
    if (fuel.m_vel.toTranslation2d().dot(normal) < 0) {
      fuel.addImpulse(
          new Translation3d(
              normal.times(-fuel.m_vel.toTranslation2d().dot(normal) * (1 + ROBOT_COR))));
    }
    if (robotVel.dot(normal) > 0) {
      fuel.addImpulse(new Translation3d(normal.times(robotVel.dot(normal))));
    }
  }

  protected void handleRobotCollisions(ArrayList<Fuel> fuels) {
    Pose2d robot = m_robotPoseSupplier.get();
    ChassisSpeeds speeds = m_robotFieldSpeedsSupplier.get();
    Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    for (Fuel fuel : fuels) {
      handleRobotCollision(fuel, robot, robotVel);
    }
  }

  protected void handleIntakes(ArrayList<Fuel> fuels) {
    Pose2d robot = m_robotPoseSupplier.get();
    for (SimIntake intake : m_intakes) {
      for (int i = fuels.size() - 1; i >= 0; i--) {
        if (intake.shouldIntake(fuels.get(i), robot)) {
          fuels.remove(i);
        }
      }
    }
  }

  protected static void fuelCollideRectangle(Fuel fuel, Translation3d start, Translation3d end) {
    if (fuel.m_pos.getZ() > end.getZ() + FUEL_RADIUS
        || fuel.m_pos.getZ() < start.getZ() - FUEL_RADIUS) {
      return; // above rectangle
    }
    double distanceToLeft = start.getX() - FUEL_RADIUS - fuel.m_pos.getX();
    double distanceToRight = fuel.m_pos.getX() - end.getX() - FUEL_RADIUS;
    double distanceToTop = fuel.m_pos.getY() - end.getY() - FUEL_RADIUS;
    double distanceToBottom = start.getY() - FUEL_RADIUS - fuel.m_pos.getY();

    // not inside hub
    if (distanceToLeft > 0 || distanceToRight > 0 || distanceToTop > 0 || distanceToBottom > 0) {
      return;
    }

    Translation2d collision;
    // find minimum distance to side and send corresponding collision response
    if (fuel.m_pos.getX() < start.getX()
        || distanceToLeft >= distanceToRight
            && distanceToLeft >= distanceToTop
            && distanceToLeft >= distanceToBottom) {
      collision = new Translation2d(distanceToLeft, 0);
    } else if (fuel.m_pos.getX() >= end.getX()
        || distanceToRight >= distanceToLeft
            && distanceToRight >= distanceToTop
            && distanceToRight >= distanceToBottom) {
      collision = new Translation2d(-distanceToRight, 0);
    } else if (fuel.m_pos.getY() > end.getY()
        || distanceToTop >= distanceToLeft
            && distanceToTop >= distanceToRight
            && distanceToTop >= distanceToBottom) {
      collision = new Translation2d(0, -distanceToTop);
    } else {
      collision = new Translation2d(0, distanceToBottom);
    }

    if (collision.getX() != 0) {
      fuel.m_pos = fuel.m_pos.plus(new Translation3d(collision));
      fuel.m_vel = fuel.m_vel.plus(new Translation3d(-(1 + FIELD_COR) * fuel.m_vel.getX(), 0, 0));
    } else if (collision.getY() != 0) {
      fuel.m_pos = fuel.m_pos.plus(new Translation3d(collision));
      fuel.m_vel = fuel.m_vel.plus(new Translation3d(0, -(1 + FIELD_COR) * fuel.m_vel.getY(), 0));
    }
  }

  /**
   * Registers an intake with the fuel simulator. This intake will remove fuel from the field based
   * on the `ableToIntake` parameter.
   *
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   * @param ableToIntake Should a return a boolean whether the intake is active
   * @param intakeCallback Function to call when a fuel is intaked
   */
  public void registerIntake(
      double xMin,
      double xMax,
      double yMin,
      double yMax,
      BooleanSupplier ableToIntake,
      Runnable intakeCallback) {
    m_intakes.add(new SimIntake(xMin, xMax, yMin, yMax, ableToIntake, intakeCallback));
  }

  /**
   * Registers an intake with the fuel simulator. This intake will remove fuel from the field based
   * on the `ableToIntake` parameter.
   *
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   * @param ableToIntake Should a return a boolean whether the intake is active
   */
  public void registerIntake(
      double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake) {
    registerIntake(xMin, xMax, yMin, yMax, ableToIntake, () -> {});
  }

  /**
   * Registers an intake with the fuel simulator. This intake will always remove fuel from the
   * field.
   *
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   * @param intakeCallback Function to call when a fuel is intaked
   */
  public void registerIntake(
      double xMin, double xMax, double yMin, double yMax, Runnable intakeCallback) {
    registerIntake(xMin, xMax, yMin, yMax, () -> true, intakeCallback);
  }

  /**
   * Registers an intake with the fuel simulator. This intake will always remove fuel from the
   * field.
   *
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   */
  public void registerIntake(double xMin, double xMax, double yMin, double yMax) {
    registerIntake(xMin, xMax, yMin, yMax, () -> true, () -> {});
  }

  /**
   * Registers an intake with the fuel simulator. This intake will remove fuel from the field based
   * on the `ableToIntake` parameter.
   *
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   * @param ableToIntake Should a return a boolean whether the intake is active
   * @param intakeCallback Function to call when a fuel is intaked
   */
  public void registerIntake(
      Distance xMin,
      Distance xMax,
      Distance yMin,
      Distance yMax,
      BooleanSupplier ableToIntake,
      Runnable intakeCallback) {
    registerIntake(
        xMin.in(Meters),
        xMax.in(Meters),
        yMin.in(Meters),
        yMax.in(Meters),
        ableToIntake,
        intakeCallback);
  }

  /**
   * Registers an intake with the fuel simulator. This intake will remove fuel from the field based
   * on the `ableToIntake` parameter.
   *
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   * @param ableToIntake Should a return a boolean whether the intake is active
   */
  public void registerIntake(
      Distance xMin, Distance xMax, Distance yMin, Distance yMax, BooleanSupplier ableToIntake) {
    registerIntake(
        xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), ableToIntake);
  }

  /**
   * Registers an intake with the fuel simulator. This intake will always remove fuel from the
   * field.
   *
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   * @param intakeCallback Function to call when a fuel is intaked
   */
  public void registerIntake(
      Distance xMin, Distance xMax, Distance yMin, Distance yMax, Runnable intakeCallback) {
    registerIntake(
        xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), intakeCallback);
  }

  /**
   * Registers an intake with the fuel simulator. This intake will always remove fuel from the
   * field.
   *
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   */
  public void registerIntake(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
    registerIntake(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
  }

  public static class Hub {
    public static final Hub BLUE_HUB =
        new Hub(
            new Translation2d(4.61, FIELD_WIDTH / 2),
            new Translation3d(5.3, FIELD_WIDTH / 2, 0.89),
            1);
    public static final Hub RED_HUB =
        new Hub(
            new Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2),
            new Translation3d(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2, 0.89),
            -1);

    protected static final double ENTRY_HEIGHT = 1.83;
    protected static final double ENTRY_RADIUS = 0.56;

    protected static final double SIDE = 1.2;

    protected static final double NET_HEIGHT_MAX = 3.057;
    protected static final double NET_HEIGHT_MIN = 1.5;
    protected static final double NET_OFFSET = SIDE / 2 + 0.261;
    protected static final double NET_WIDTH = 1.484;

    protected final Translation2d m_center;
    protected final Translation3d m_exit;
    protected final int m_exitVelXMult;

    protected int m_score;

    protected Hub(Translation2d center, Translation3d exit, int exitVelXMult) {
      this.m_center = center;
      this.m_exit = exit;
      this.m_exitVelXMult = exitVelXMult;
    }

    protected void handleHubInteraction(Fuel fuel, int subticks) {
      if (didFuelScore(fuel, subticks)) {
        fuel.m_pos = m_exit;
        fuel.m_vel = getDispersalVelocity();
        m_score++;
      }
    }

    protected boolean didFuelScore(Fuel fuel, int subticks) {
      return fuel.m_pos.toTranslation2d().getDistance(m_center) <= ENTRY_RADIUS
          && fuel.m_pos.getZ() <= ENTRY_HEIGHT
          && fuel.m_pos.minus(fuel.m_vel.times(PERIOD / subticks)).getZ() > ENTRY_HEIGHT;
    }

    protected Translation3d getDispersalVelocity() {
      return new Translation3d(
          m_exitVelXMult * (Math.random() + 0.1) * 1.5, Math.random() * 2 - 1, 0);
    }

    /** Resets this hub's score to 0. */
    public void resetScore() {
      m_score = 0;
    }

    /**
     * Gets the current count of fuel scored in this hub.
     *
     * @return the score count
     */
    public int getScore() {
      return m_score;
    }

    protected void fuelCollideSide(Fuel fuel) {
      fuelCollideRectangle(
          fuel,
          new Translation3d(m_center.getX() - SIDE / 2, m_center.getY() - SIDE / 2, 0),
          new Translation3d(
              m_center.getX() + SIDE / 2, m_center.getY() + SIDE / 2, ENTRY_HEIGHT - 0.1));
    }

    protected double fuelHitNet(Fuel fuel) {
      if (fuel.m_pos.getZ() > NET_HEIGHT_MAX || fuel.m_pos.getZ() < NET_HEIGHT_MIN) {
        return 0;
      }
      if (fuel.m_pos.getY() > m_center.getY() + NET_WIDTH / 2
          || fuel.m_pos.getY() < m_center.getY() - NET_WIDTH / 2) {
        return 0;
      }
      if (fuel.m_pos.getX() > m_center.getX() + NET_OFFSET * m_exitVelXMult) {
        return Math.max(
            0, m_center.getX() + NET_OFFSET * m_exitVelXMult - (fuel.m_pos.getX() - FUEL_RADIUS));
      } else {
        return Math.min(
            0, m_center.getX() + NET_OFFSET * m_exitVelXMult - (fuel.m_pos.getX() + FUEL_RADIUS));
      }
    }
  }

  protected class SimIntake {
    double m_xMin;
    double m_xMax;
    double m_yMin;
    double m_yMax;
    BooleanSupplier m_ableToIntake;
    Runnable m_callback;

    protected SimIntake(
        double xMin,
        double xMax,
        double yMin,
        double yMax,
        BooleanSupplier ableToIntake,
        Runnable intakeCallback) {
      this.m_xMin = xMin;
      this.m_xMax = xMax;
      this.m_yMin = yMin;
      this.m_yMax = yMax;
      this.m_ableToIntake = ableToIntake;
      this.m_callback = intakeCallback;
    }

    protected boolean shouldIntake(Fuel fuel, Pose2d robotPose) {
      if (!m_ableToIntake.getAsBoolean() || fuel.m_pos.getZ() > m_bumperHeight) {
        return false;
      }

      Translation2d fuelRelativePos =
          new Pose2d(fuel.m_pos.toTranslation2d(), Rotation2d.kZero)
              .relativeTo(robotPose)
              .getTranslation();

      boolean result =
          fuelRelativePos.getX() >= m_xMin
              && fuelRelativePos.getX() <= m_xMax
              && fuelRelativePos.getY() >= m_yMin
              && fuelRelativePos.getY() <= m_yMax;
      if (result) {
        m_callback.run();
      }
      return result;
    }
  }
}
