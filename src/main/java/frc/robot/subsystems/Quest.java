package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Quest extends SubsystemBase {
  private final QuestNav questNav = new QuestNav();

  private Pose2d latestPose = new Pose2d();
  private double latestTimestamp = 0;

  private final Field2d questField = new Field2d();

  // Tunables
  private double offsetX = -0.600;
  private double offsetY = 0.0;
  private double offsetRotationDeg = 180.0;
  private boolean headingInputIsDegrees = true;

  public Quest() {
    SmartDashboard.putNumber("Quest2/OffsetX_m", offsetX);
    SmartDashboard.putNumber("Quest2/OffsetY_m", offsetY);
    SmartDashboard.putNumber("Quest2/Rotation_deg", offsetRotationDeg);
    SmartDashboard.putBoolean("Quest2/HeadingInputIsDegrees", headingInputIsDegrees);
  }

  private Rotation2d decodeQuestHeading(Pose2d rawPose) {
    final double rawYaw = rawPose.getRotation().getRadians();
    return headingInputIsDegrees
        ? Rotation2d.fromDegrees(rawYaw)
        : Rotation2d.fromRadians(rawYaw);
  }

  private Pose2d encodeRobotPoseForQuest(Pose2d robotPose) {
    if (!headingInputIsDegrees) {
      return robotPose;
    }

    final double robotHeadingDeg = robotPose.getRotation().getDegrees();
    return new Pose2d(
        robotPose.getX(),
        robotPose.getY(),
        Rotation2d.fromRadians(robotHeadingDeg)
    );
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    // Get tunable values FIRST
    offsetX = SmartDashboard.getNumber("Quest2/OffsetX_m", offsetX);
    offsetY = SmartDashboard.getNumber("Quest2/OffsetY_m", offsetY);
    offsetRotationDeg = SmartDashboard.getNumber("Quest2/Rotation_deg", offsetRotationDeg);
    headingInputIsDegrees = SmartDashboard.getBoolean("Quest2/HeadingInputIsDegrees", headingInputIsDegrees);

    PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
    if (frames.length > 0) {
      Pose2d unreadRawPose = frames[frames.length - 1].questPose3d().toPose2d();
      latestTimestamp = frames[frames.length - 1].dataTimestamp();

      Pose2d rawPose = new Pose2d(
          unreadRawPose.getTranslation(),
          decodeQuestHeading(unreadRawPose)
      );
      
      // Build transform with current tunables
      Transform2d robotToQuest = new Transform2d(
          offsetX, 
          offsetY, 
          Rotation2d.fromDegrees(offsetRotationDeg)
      );
      
      // Transform ONCE and use for both latestPose and field
      latestPose = rawPose.transformBy(robotToQuest.inverse());
      questField.setRobotPose(latestPose);

      // Debug logging
      SmartDashboard.putString("Quest2/RAW Quest Pose", rawPose.toString());
      SmartDashboard.putNumber("Quest2/RAW X", rawPose.getX());
      SmartDashboard.putNumber("Quest2/RAW Y", rawPose.getY());
      SmartDashboard.putNumber("Quest2/RAW Rotation (deg)", rawPose.getRotation().getDegrees());
  SmartDashboard.putNumber("Quest2/RAW Rotation Source Value", unreadRawPose.getRotation().getRadians());
      
      SmartDashboard.putString("Quest2/TRANSFORMED Robot Pose", latestPose.toString());
      SmartDashboard.putNumber("Quest2/TRANSFORMED X (m)", latestPose.getX());
      SmartDashboard.putNumber("Quest2/TRANSFORMED Y (m)", latestPose.getY());
      SmartDashboard.putNumber("Quest2/TRANSFORMED Heading (deg)", latestPose.getRotation().getDegrees());
    }

    SmartDashboard.putBoolean("Quest2/Connected", isConnected());
    SmartDashboard.putBoolean("Quest2/Tracking", isTracking());
    SmartDashboard.putNumber("Quest2/Battery %", getBatteryPercent());
    SmartDashboard.putNumber("Quest2/Tracking Losses", getTrackingLostCount());
    SmartDashboard.putNumber("Quest2/Pose Age (s)", Timer.getFPGATimestamp() - latestTimestamp);
    SmartDashboard.putNumber("Quest2/PoseFrame Count", frames.length);
    SmartDashboard.putData("Quest2/Field", questField);
  }

  public Pose2d getLatestPose() {
    return latestPose;
  }

  public double getLatestTimestamp() {
    return latestTimestamp;
  }

  public boolean hasFreshPose() {
    return Timer.getFPGATimestamp() - latestTimestamp < 0.3;
  }

  public void setPose(Pose2d robotPose) {
    double currentOffsetX = SmartDashboard.getNumber("Quest2/OffsetX_m", offsetX);
    double currentOffsetY = SmartDashboard.getNumber("Quest2/OffsetY_m", offsetY);
    double currentOffsetRotation = SmartDashboard.getNumber("Quest2/Rotation_deg", offsetRotationDeg);
    headingInputIsDegrees = SmartDashboard.getBoolean("Quest2/HeadingInputIsDegrees", headingInputIsDegrees);
    
    Transform2d robotToQuest = new Transform2d(
        currentOffsetX, 
        currentOffsetY, 
        Rotation2d.fromDegrees(currentOffsetRotation)
    );
    
    
    Pose2d questPose2d = encodeRobotPoseForQuest(robotPose).transformBy(robotToQuest);
    Pose3d questPose3d = new Pose3d(questPose2d);
    questNav.setPose(questPose3d);
  }

  public boolean isConnected() {
    return questNav.isConnected();
  }

  public boolean isTracking() {
    return questNav.isTracking();
  }

  public int getBatteryPercent() {
    return questNav.getBatteryPercent().getAsInt();
  }

  public int getTrackingLostCount() {
    return questNav.getTrackingLostCounter().getAsInt();
  }
}