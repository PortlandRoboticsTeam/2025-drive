// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ArmPosition;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.subsystems.Motor.MotorType;
import frc.robot.subsystems.Encoder.EncoderType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 1;
    public static final int helperControllerPort = 0;

    public static final String AutonomousCommandName = "test";
  }

  public static final double DEADBAND = 0.1;
  public static final double maximumSpeed = Units.feetToMeters(4.5);
  public static final TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.HIGH;
  
  public static class GrabberConstants {
    public static final int coralMotorID = 17;
    public static final int algaeMotorID = 18;
    public static final double coralSpeed = .2;
    public static final double algaeSpeed = 0.3;
    public static final MotorType algaeMotorType = MotorType.Ghost;
    public static final MotorType coralMotorType = MotorType.Ghost;
  }
  public static class ArmConstants {
    public static final double manualControlJoystickDeaband = 0.1;

    public static final double shoulderOffset = 27, shoulderMin =  0, shoulderMax = 184;
    public static final int    shoulder1ID    = 13, shoulder2ID = 14;
    public static final int shoulderEncoderID  = 0;
    public static final MotorType shoulder1Type = MotorType.SparkMax;
    public static final MotorType shoulder2Type = MotorType.SparkMax;
    public static final EncoderType shoulderEncoderType = EncoderType.DutyCycle;


    public static final double wristOffset = 0, wristMin =  0, wristMax = 0;
    public static final int wristID  = 31;
    public static final int wristEncoderID  = 0;
    public static final MotorType wristType = MotorType.Ghost;
    public static final EncoderType wristEncoderType = EncoderType.Ghost;


    public static final double telescopeOffset = 0, telescopeMin =  0, telescopeMax = 744;
    public static final int telescopeID  = 15;
    public static final int telescopeEncoderID  = 16;
    public static final int greenThreshold = 15000;
    public static final MotorType telescopeType = MotorType.SparkMax;
    public static final EncoderType telescopeEncoderType = EncoderType.CANCoder;

    public static final ArmPosition positionCommandCompletionTolerance = new ArmPosition(5, .2, 1, "tolerance (not a position)");


    public static final ArmPosition[] positions = {
      new ArmPosition(0, 0, 0, "Rest: 0"),

      new ArmPosition(0, 0, 0, "Tray: 1"),
      new ArmPosition(0, 0, 0, "Low Right Coral: 2"),
      new ArmPosition(0, 0, 0, "Low Left Coral: 3"),
      new ArmPosition(0, 0, 0, "Low Reef Ball: 4"),
      new ArmPosition(0, 0, 0, "Mid Right Coral: 5"),
      new ArmPosition(0, 0, 0, "Mid Left Coral: 6"),
      new ArmPosition(0, 0, 0, "High Reef Ball: 7"),
      new ArmPosition(78, 0, 0, "High Right Coral: 8"),
      new ArmPosition(78, 0, 0, "High Left Coral: 9"),

      new ArmPosition(0, 0, 0, "Ready to Climb: 10"),
      new ArmPosition(122, 0, 0, "Climbing: 11"),
      new ArmPosition(0, 0, 0, "Collect Stacked Algae: 12"),
      new ArmPosition(0, 0, 0, "Collect Standing Coral: 13"),
      new ArmPosition(0, 0, 0, "Collect Floor Algae: 14"),
      new ArmPosition(0, 0, 0, "Collect Floor Coral: 15"),

      new ArmPosition(0, 0, 0, "Collect Coral Human: 16"),
      new ArmPosition(0, 0, 0, "Score Algae to Human: 17"),
      new ArmPosition(90, 0, 0, "Score Algae to Barge: 18"),
    };    
  }
}
