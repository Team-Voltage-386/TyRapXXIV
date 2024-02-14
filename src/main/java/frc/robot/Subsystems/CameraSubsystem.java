// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.LimelightHelpers;

public class CameraSubsystem extends SubsystemBase {
  UsbCamera usbCamera;
  NetworkTableEntry cameraSelection;
  VideoSink server;

    ShuffleboardTab m_cameraTab = Shuffleboard.getTab("camera tab");
    private final SimpleWidget m_llTimeSinceUpdateOdo = m_cameraTab.add("Time Since LL Odo Update", 0);
    private Timer m_llTimeSinceUpdate = new Timer();
    private final SimpleWidget m_limelightPose2DBlue = m_cameraTab.add("getBotPose2DwpiBlue", "");
    private ComplexWidget m_cameraView;
    private static String limelightName = "limelight-b";

  public CameraSubsystem() {
    // Creates UsbCamera and MjpegServer [1] and connects them
    this.usbCamera = CameraServer.startAutomaticCapture("USB camera", 0);
    this.server = CameraServer.getServer();

    this.cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

    this.usbCamera.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);

    this.setCameraSource(CameraSourceOption.USB_CAMERA);

    this.usbCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    m_cameraView = m_cameraTab.add(this.usbCamera).withWidget(BuiltInWidgets.kCameraStream).withPosition(3, 0).withSize(6, 4);
    
    m_llTimeSinceUpdate.start();
  }

  public static String getLimelightName() {
      return limelightName;
  }

  public static enum CameraSourceOption {
    USB_CAMERA,
  }

  public void setCameraSource(CameraSourceOption option) {
    switch(option) {
      case USB_CAMERA: {
        this.server.setSource(this.usbCamera);
        break;
      }
      default: {
        break;
      }
    }
  }

  public Pose2d resetOdoLimelight() {
        m_limelightPose2DBlue.getEntry().setString(LimelightHelpers.getBotPose2d_wpiBlue(limelightName).toString());
        LimelightHelpers.LimelightResults  llResults = LimelightHelpers.getLatestResults(limelightName);
        int numTargets = llResults.targetingResults.targets_Fiducials.length;
        m_llTimeSinceUpdateOdo.getEntry().setDouble(m_llTimeSinceUpdate.get());
        if (numTargets > 1) {
          m_llTimeSinceUpdate.reset();
          return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
        }
        else{
          return null;
        } 
    }

  public boolean isLLOdoGood(double timeThreshold) {
      return !m_llTimeSinceUpdate.hasElapsed(timeThreshold);
  }
}
