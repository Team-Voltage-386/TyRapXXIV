// package frc.robot.ShuffleboardLayouts;

// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// public class DrivetrainLayout {
//     ShuffleboardTab driveTab;
//     private GenericEntry desiredXSpeed;
//     private GenericEntry desiredYSpeed;
//     private GenericEntry desiredRotSpeed;
//     private GenericEntry limelightFRTargetAngle;
//     private GenericEntry limelightRRTargetAngle;
//     private GenericEntry limelightTagID;
//     private GenericEntry actualAngleToSpeaker;
//     private GenericEntry desiredAngleToSpeaker;

//     public DrivetrainLayout() {
//         driveTab = Shuffleboard.getTab("Drive tab");
//         desiredXSpeed = driveTab.add("Desired X Speed", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
//         desiredYSpeed = driveTab.add("Desired Y Speed", 0.0).withPosition(1, 0).withSize(1, 1).getEntry();
//         desiredRotSpeed = driveTab.add("Desired Rot Speed", 0.0).withPosition(2, 0).withSize(2, 1).getEntry();
//         limelightFRTargetAngle = driveTab.add("Field Relative Target Angle", 0.0)
//                 .withPosition(0, 2).withSize(2, 1).getEntry();
//         limelightRRTargetAngle = driveTab.add("Robot Relative Target Angle", 0.0)
//                 .withPosition(2, 2).withSize(2, 1).getEntry();
//         limelightTagID = driveTab.add("Seen Tag ID", 0.0)
//                 .withPosition(5, 2).withSize(1, 1).getEntry();
//         actualAngleToSpeaker = driveTab.add("Actual Angle to Speaker", 0.0)
//                 .withPosition(0, 4).withSize(2, 1).getEntry();
//         desiredAngleToSpeaker = driveTab.add("Desired Angle to Speaker", 0.0)
//                 .withPosition(2, 4).withSize(2, 1).getEntry();
//     }

//     public GenericEntry getDesiredXSpeed() {
//         return desiredXSpeed;
//     }

//     public void setDesiredXSpeed(double desiredXSpeed) {
//         this.desiredXSpeed.setDouble(desiredXSpeed);
//     }

//     public GenericEntry getDesiredYSpeed() {
//         return desiredYSpeed;
//     }

//     public void setDesiredYSpeed(double desiredYSpeed) {
//         this.desiredYSpeed.setDouble(desiredYSpeed);
//     }

//     public GenericEntry getDesiredRotSpeed() {
//         return desiredRotSpeed;
//     }

//     public void setDesiredRotSpeed(double desiredRotSpeed) {
//         this.desiredRotSpeed.setDouble(desiredRotSpeed);
//     }

//     public GenericEntry getLimelightFRTargetAngle() {
//         return limelightFRTargetAngle;
//     }

//     public void setLimelightFRTargetAngle(double limelightFRTargetAngle) {
//         this.limelightFRTargetAngle.setDouble(limelightFRTargetAngle);
//     }

//     public GenericEntry getLimelightRRTargetAngle() {
//         return limelightRRTargetAngle;
//     }

//     public void setLimelightRRTargetAngle(double limelightRRTargetAngle) {
//         this.limelightRRTargetAngle.setDouble(limelightRRTargetAngle);
//     }

//     public GenericEntry getLimelightTagID() {
//         return limelightTagID;
//     }

//     public void setLimelightTagID(double limelightTagID) {
//         this.limelightTagID.setDouble(limelightTagID);
//     }

//     public GenericEntry getActualAngleToSpeaker() {
//         return actualAngleToSpeaker;
//     }

//     public void setActualAngleToSpeaker(double actualAngleToSpeaker) {
//         this.actualAngleToSpeaker.setDouble(actualAngleToSpeaker);
//     }

//     public GenericEntry getDesiredAngleToSpeaker() {
//         return desiredAngleToSpeaker;
//     }

//     public void setDesiredAngleToSpeaker(double desiredAngleToSpeaker) {
//         this.desiredAngleToSpeaker.setDouble(desiredAngleToSpeaker);
//     }

    
// }
