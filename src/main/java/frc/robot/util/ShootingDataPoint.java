package frc.robot.util;

public record ShootingDataPoint(
    double distance, double bottomRpm, double topRollerRpm, Double tof) {
  public ShootingDataPoint(double distance, double bottomRpm, double topRpm) {
    this(distance, bottomRpm, topRpm, null);
  }
}
