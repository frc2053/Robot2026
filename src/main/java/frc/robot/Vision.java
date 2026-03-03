/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot;

import static frc.robot.Constants.VisionConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class Vision {
  private final Camera m_frontCamera;

  /**
   * Creates a new Vision instance.
   *
   * @param estConsumer Lambda that will accept a pose estimate and pass it to your desired {@link
   *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
   */
  public Vision(Camera.VisionEstimateConsumer estConsumer) {
    m_frontCamera =
        new Camera(
            VisionConstants.kFrontCameraName,
            VisionConstants.kFrontRobotToCam,
            VisionConstants.kSingleTagStdDevs,
            VisionConstants.kMultiTagStdDevs,
            true,
            estConsumer);
  }

  /**
   * Updates pose estimators for both cameras.
   *
   * @param robotPose The current robot pose for visualization purposes.
   */
  public void periodic(Pose3d robotPose) {
    m_frontCamera.updatePoseEstimator(robotPose);
  }

  /**
   * Updates the vision simulation with the robot pose.
   *
   * @param robotSimPose The simulated robot pose.
   */
  public void simulationPeriodic(Pose2d robotSimPose) {
    m_frontCamera.simPeriodic(robotSimPose);
  }
}
