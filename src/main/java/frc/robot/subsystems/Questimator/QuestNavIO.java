// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Questimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface QuestNavIO {
  // Modified to add pitch and 
  @AutoLog
  public static class QuestimatorIOInputs {
    public Pose2d questPose;
    public Pose2d robotPose;
    public double battery;
    public boolean connected = false;
    public Quaternion quaternion;
    public double timestamp;
  }

  public default void updateInputs(QuestimatorIOInputs inputs) {}
  public default Pose2d getQuestPose(){return null;}
  public default Pose2d getRobotPose(){return null;}
  // Gets the battery percent of the Quest.
  public default double getBatteryPercent(){return 0;}
  // Returns if the Quest is connected.
  public default boolean connected() {return false;}
  // Gets the Quaternion of the Quest.
  public default Quaternion getQuaternion(){return null;}
  // Gets the Quests's timestamp.
  public default double timestamp() {return 0;}
  // Zero the relativerobot heading
  public default void zeroHeading(){}
  // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
  public default void zeroPosition(){}
  // public void setPosition(Pose2d newPose) {
  public default void setPosition(Pose2d pose){}
  public default void resetPose(Pose2d newPose){}
  // Clean up questnav subroutine messages after processing on the headset
  public default void cleanUpQuestNavMessages(){}
  // Get the yaw Euler angle of the headset
  public default float getOculusYaw() {return 0;}
  public default Translation2d getQuestNavTranslation(){return null;}
  public default Pose2d getQuestNavPose(){return null;}
  public default void hardReset() {}
  
}