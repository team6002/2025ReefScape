package frc.robot.subsystems.Questimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class QuestNavIOMeta implements QuestNavIO{
  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  // Local heading helper variables
  private float yaw_offset = 0.0f;
  private Pose2d resetPosition = new Pose2d();
  private Pose2d resetPositionOculus = new Pose2d();
  private Pose2d resetPositionRobot = new Pose2d();
  private double postitionModX = 1.036;//1.046;//1.036;
  private double postitionModY = 1.047;//1.069;//1.036;

  private Transform2d kRobotToQuest = new Transform2d(Units.inchesToMeters(3.5),Units.inchesToMeters(-7), new Rotation2d(-Math.PI/2));
  // private Transform2d kRobotToQuest = new Transform2d(0.06,-0.22, new Rotation2d(-Math.PI/2));
  // private Transform2d kRobotToQuest = new Transform2d(0.0,-0., new Rotation2d(-Math.PI/2));
  // Gets the Quest's measured position.
  @Override
  public Pose2d getQuestPose() {
    // Pose2d rawPose = new Pose2d(getQuestNavTranslation(), new Rotation2d(Math.toRadians(getOculusYaw())));
    var rawPose = getUncorrectedOculusPose();
    var poseRelativetoReset = rawPose.minus(resetPositionOculus);
    return resetPositionRobot.transformBy(poseRelativetoReset);
    // return resetPositionRobot.transformBy(kRobotToQuest);
    // return new Pose2d(getQuestNavPose().minus(resetPosition).getTranslation(), Rotation2d.fromDegrees(getOculusYaw()));
  }

  @Override
  public Pose2d getRobotPose(){
    Pose2d translationPose = new Pose2d(getQuestPose().getX() * postitionModX, getQuestPose().getY() * postitionModY, new Rotation2d());
    return new Pose2d(translationPose.getTranslation(), getQuestPose().getRotation()).transformBy(kRobotToQuest.inverse());
    // (getPose().getX() - Units.inchesToMeters(8) * Math.cos())
  }

  // Gets the battery percent of the Quest.
  @Override
  public double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  // Returns if the Quest is connected.
  @Override
  public boolean connected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  // Gets the Quaternion of the Quest.
  @Override
  public Quaternion getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
  }

  // Gets the Quests's timestamp.
  @Override
  public double timestamp() {
    return questTimestamp.get();
  }

  // Zero the relativerobot heading
  @Override
  public void zeroHeading() {
    yaw_offset = 0;
    float[] eulerAngles = questEulerAngles.get();
    yaw_offset = eulerAngles[1];
  }

  // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
  @Override
  public void zeroPosition() {
    resetPosition = new Pose2d(0,0,new Rotation2d());
    resetPosition = getQuestPose();
    // if (questMiso.get() != 99) {
    //   questMosi.set(1);
    // }
  }

  // @Override
  // public void setPosition(Pose2d newPose) {
  //   resetPosition = new Pose2d(0, 0, new Rotation2d());
  //   resetPosition = getQuestPose();
  //   resetPosition = newPose.transformBy(kRobotToQuest.inverse());
  //   // restPoseOcculus
  //   // if (questMiso.get() != 99) {
  //   //   questMosi.set(1);
  //   // }
  // }

  @Override
  public void resetPose(Pose2d newPose){
    resetPosition = new Pose2d(0,0,new Rotation2d());
    resetPositionOculus = getUncorrectedOculusPose().transformBy(kRobotToQuest.inverse());
    // Pose2d uncorrectedOcculusPose = new Pose2d(getQuestNavTranslation(), new Rotation2d(Math.toRadians(getOculusYaw())));
    // resetPositionOculus = uncorrectedOcculusPose.transformBy(kRobotToQuest.inverse());
    // var transformationerToCenter = (newPose.minus(new Pose2d(.03, .17, new Rotation2d()))); 
    // var reseterPose = new Pose2d (transformationerToCenter.getX(), transformationerToCenter.getY(), transformationerToCenter.getRotation());
    // resetPositionRobot = reseterPose;
    Pose2d translationPose = new Pose2d(newPose.getX()/postitionModX, newPose.getY()/postitionModY, new Rotation2d());
    resetPositionRobot = new Pose2d(translationPose.getTranslation(), newPose.getRotation());

  }
  // Clean up questnav subroutine messages after processing on the headset
  @Override
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  // Get the yaw Euler angle of the headset
  @Override
  public float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    var ret = eulerAngles[1];
    //  - yaw_offset;
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    return -ret;
  }

  @Override
  public Translation2d getQuestNavTranslation() {
    float[] questnavPosition = questPosition.get();
    return new Translation2d(questnavPosition[2], -questnavPosition[0]);
  }

  @Override
  public Pose2d getQuestNavPose() {
    var oculousPositionCompensated = getQuestNavTranslation().minus(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0))); // 6.5
    return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
  }

  private Pose2d getUncorrectedOculusPose() {
    var eulerAngles = questEulerAngles.get();
    var rotation = Rotation2d.fromDegrees(-Math.IEEEremainder(eulerAngles[1], 360d));

    var questnavPosition = questPosition.get();
    var translation = new Translation2d(questnavPosition[2], -questnavPosition[0]);
    return new Pose2d(translation, rotation);
  }

  @Override
  public void updateInputs(QuestimatorIOInputs inputs){
    inputs.questPose = getQuestPose();
    inputs.robotPose = getRobotPose();
    inputs.battery = getBatteryPercent();
    inputs.connected = connected();
    inputs.quaternion = getQuaternion();
    inputs.timestamp = timestamp();
  }

  @Override
  public void hardReset() {
    // resetPosition = getPose();
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }
}