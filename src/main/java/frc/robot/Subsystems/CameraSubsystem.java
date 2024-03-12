// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.LimelightHelpers;

public class CameraSubsystem extends SubsystemBase {
  HttpCamera limeLightCamera;
  VideoSink server;

  ShuffleboardTab m_cameraTab = Shuffleboard.getTab("camera tab");
  private final SimpleWidget m_llTimeSinceUpdateOdo = m_cameraTab.add("Time Since LL Odo Update", 0);
  private Timer m_llTimeSinceUpdate = new Timer();
  private final SimpleWidget m_limelightPose2DBlue = m_cameraTab.add("getBotPose2DwpiBlue", "");
  private ComplexWidget m_cameraView;
  private static String shooterLimeLightName = "limelight-b";
  private static String shooterLimeLightHttp = "http://10.3.86.12";

  private static String pickupLimeLightName = "limelight-c";
  private static String pickupLimeLightHttp = "http://10.3.86.13";

  ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition Tab");
  private ComplexWidget m_competitionCameraView;

  public CameraSubsystem() {
    // Creates UsbCamera and MjpegServer [1] and connects them\
    this.limeLightCamera = new HttpCamera(pickupLimeLightName, pickupLimeLightHttp + ":5800/stream.mjpg",
        HttpCameraKind.kMJPGStreamer);
    CameraServer.startAutomaticCapture(this.limeLightCamera);
    this.server = CameraServer.getServer();
    this.server.setSource(this.limeLightCamera);

    m_cameraView = m_cameraTab.add(this.limeLightCamera).withWidget(BuiltInWidgets.kCameraStream).withPosition(3, 0)
        .withSize(6, 4);

    m_competitionCameraView = m_competitionTab.add(this.limeLightCamera).withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(0, 0).withSize(5, 5);

    m_llTimeSinceUpdate.start();
  }

  public static String getShooterLimelightName() {
    return shooterLimeLightName;
  }

  public static String getPickupLimelightName() {
    return pickupLimeLightName;
  }

  public Pose2d resetOdoLimelight() {
    LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(shooterLimeLightName);
    m_limelightPose2DBlue.getEntry().setString(poseEstimate.toString());
    m_llTimeSinceUpdateOdo.getEntry().setDouble(m_llTimeSinceUpdate.get());
    if (poseEstimate.tagCount > 1) {
      m_llTimeSinceUpdate.reset();
      return poseEstimate.pose;
    } else {
      return null;
    }
  }

  public boolean isLLOdoGood(double timeThreshold) {
    return !m_llTimeSinceUpdate.hasElapsed(timeThreshold);
  }
}
