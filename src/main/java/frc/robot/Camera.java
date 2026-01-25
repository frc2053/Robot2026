// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Camera class for handling individual PhotonVision cameras with pose estimation. */
public class Camera {
  private final boolean m_simulate;
  private final VisionEstimateConsumer m_consumer;
  private final Matrix<N3, N1> m_singleTagDevs;
  private final Matrix<N3, N1> m_multiTagDevs;

  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_photonEstimator;

  // NetworkTables publishers
  private final NetworkTable m_nt;
  private final StructPublisher<Pose2d> m_posePub;
  private final DoublePublisher m_stdDevXPosePub;
  private final DoublePublisher m_stdDevYPosePub;
  private final DoublePublisher m_stdDevRotPosePub;
  private final StructArrayPublisher<Pose3d> m_targetPosesPub;
  private final StructArrayPublisher<Translation2d> m_cornersPub;

  // Simulation
  private VisionSystemSim m_visionSim;

  // Cache for targets
  private List<PhotonTrackedTarget> m_targetsCopy = new ArrayList<>();

  /**
   * Creates a new Camera instance.
   *
   * @param cameraName The name of the PhotonVision camera.
   * @param robotToCamera The transform from the robot center to the camera.
   * @param singleTagStdDev The standard deviations for single tag estimates [x, y, rotation].
   * @param multiTagDevs The standard deviations for multi tag estimates [x, y, rotation].
   * @param simulate Whether to enable simulation support.
   * @param visionConsumer Consumer for vision estimates.
   */
  public Camera(
      String cameraName,
      Transform3d robotToCamera,
      Matrix<N3, N1> singleTagStdDev,
      Matrix<N3, N1> multiTagDevs,
      boolean simulate,
      VisionEstimateConsumer visionConsumer) {
    this.m_simulate = simulate;
    this.m_consumer = visionConsumer;
    this.m_singleTagDevs = singleTagStdDev;
    this.m_multiTagDevs = multiTagDevs;

    // Initialize NetworkTables
    m_nt = NetworkTableInstance.getDefault().getTable("Vision");
    m_posePub = m_nt.getStructTopic(cameraName + "PoseEstimation", Pose2d.struct).publish();
    m_stdDevXPosePub = m_nt.getDoubleTopic(cameraName + "StdDevsX").publish();
    m_stdDevYPosePub = m_nt.getDoubleTopic(cameraName + "StdDevsY").publish();
    m_stdDevRotPosePub = m_nt.getDoubleTopic(cameraName + "StdDevsRot").publish();
    m_targetPosesPub =
        m_nt.getStructArrayTopic(cameraName + "targetPoses", Pose3d.struct).publish();
    m_cornersPub =
        m_nt.getStructArrayTopic(cameraName + "targetCorners", Translation2d.struct).publish();

    // Initialize pose estimator
    m_photonEstimator =
        new PhotonPoseEstimator(Constants.VisionConstants.kTagLayout, robotToCamera);

    // Initialize camera
    m_camera = new PhotonCamera(cameraName);

    // Initialize simulation if needed
    if (simulate && RobotBase.isSimulation()) {
      m_visionSim = new VisionSystemSim(cameraName);
      m_visionSim.addAprilTags(Constants.VisionConstants.kTagLayout);

      SimCameraProperties cameraProps = new SimCameraProperties();
      cameraProps.setCalibration(1600, 1304, Rotation2d.fromDegrees(55));
      cameraProps.setCalibError(0.35, 0.10);
      cameraProps.setFPS(25.5);
      cameraProps.setAvgLatencyMs(50);
      cameraProps.setLatencyStdDevMs(5);

      PhotonCameraSim cameraSim = new PhotonCameraSim(m_camera, cameraProps);
      m_visionSim.addCamera(cameraSim, robotToCamera);
      cameraSim.enableDrawWireframe(true);
    }
  }

  /**
   * Updates the pose estimator with new camera results.
   *
   * @param robotPose The current robot pose for visualization purposes.
   */
  public void updatePoseEstimator(Pose3d robotPose) {
    List<PhotonPipelineResult> allUnread = m_camera.getAllUnreadResults();

    for (PhotonPipelineResult result : allUnread) {
      // Try multi-tag estimation first (most accurate)
      Optional<EstimatedRobotPose> visionEst = m_photonEstimator.estimateCoprocMultiTagPose(result);

      // Fallback to lowest ambiguity single-tag if multi-tag unavailable
      if (visionEst.isEmpty()) {
        visionEst = m_photonEstimator.estimateLowestAmbiguityPose(result);
      }

      // Publish pose
      if (visionEst.isPresent()) {
        m_posePub.set(visionEst.get().estimatedPose.toPose2d());
      } else {
        m_posePub.set(new Pose2d());
      }

      // Cache targets for standard deviation calculation
      m_targetsCopy = new ArrayList<>(result.getTargets());

      // Publish target poses and corners
      List<Pose3d> targetPoses = new ArrayList<>();
      List<Translation2d> cornerPxs = new ArrayList<>();
      for (PhotonTrackedTarget target : m_targetsCopy) {
        targetPoses.add(
            robotPose
                .transformBy(m_photonEstimator.getRobotToCameraTransform())
                .transformBy(target.getBestCameraToTarget()));

        for (var corner : target.getDetectedCorners()) {
          cornerPxs.add(new Translation2d(corner.x, corner.y));
        }
      }
      m_targetPosesPub.set(targetPoses.toArray(new Pose3d[0]));
      m_cornersPub.set(cornerPxs.toArray(new Translation2d[0]));

      // Send estimates to consumer
      if (visionEst.isPresent()) {
        Matrix<N3, N1> stdDevs = getEstimationStdDevs(visionEst.get().estimatedPose.toPose2d());
        m_consumer.accept(
            visionEst.get().estimatedPose.toPose2d(), visionEst.get().timestampSeconds, stdDevs);
      }
    }
  }

  /**
   * Calculates standard deviations for the estimated pose based on number of tags and distance.
   *
   * @param estimatedPose The estimated pose.
   * @return The standard deviations as a 3x1 matrix [x, y, rotation].
   */
  private Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    Matrix<N3, N1> estStdDevs = m_singleTagDevs;
    int numTags = 0;
    double avgDist = 0.0;

    // Calculate average distance to visible tags
    for (PhotonTrackedTarget target : m_targetsCopy) {
      Optional<Pose3d> tagPose =
          m_photonEstimator.getFieldTags().getTagPose(target.getFiducialId());
      if (tagPose.isPresent()) {
        numTags++;
        avgDist +=
            tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
      }
    }

    if (numTags == 0) {
      return estStdDevs;
    }

    avgDist /= numTags;

    // Use multi-tag standard deviations if multiple tags visible
    if (numTags > 1) {
      estStdDevs = m_multiTagDevs;
    }

    // Reject single tag estimates at long distance
    if (numTags == 1 && avgDist > 4.0) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      // Scale standard deviations based on distance
      estStdDevs = estStdDevs.times(1.0 + (avgDist * avgDist / 30.0));
    }

    // Log error if standard deviations are zero
    if (estStdDevs.get(0, 0) == 0 || estStdDevs.get(1, 0) == 0 || estStdDevs.get(2, 0) == 0) {
      DataLogManager.log("ERROR: STD DEV IS ZERO!");
    }

    // Publish standard deviations
    m_stdDevXPosePub.set(estStdDevs.get(0, 0));
    m_stdDevYPosePub.set(estStdDevs.get(1, 0));
    m_stdDevRotPosePub.set(estStdDevs.get(2, 0));

    return estStdDevs;
  }

  /**
   * Updates the vision simulation with the robot pose.
   *
   * @param robotSimPose The simulated robot pose.
   */
  public void simPeriodic(Pose2d robotSimPose) {
    if (m_simulate && m_visionSim != null) {
      m_visionSim.update(robotSimPose);
    }
  }

  /** Functional interface for consuming vision estimates. */
  @FunctionalInterface
  public interface VisionEstimateConsumer {
    /**
     * Accepts a vision estimate.
     *
     * @param pose The estimated pose.
     * @param timestamp The timestamp of the estimate in seconds.
     * @param stdDevs The standard deviations [x, y, rotation].
     */
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
  }
}
