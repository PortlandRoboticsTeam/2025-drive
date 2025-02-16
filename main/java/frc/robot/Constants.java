// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ArmPosition;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

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
    public static final int driverControllerPort = 0;
    public static final int helperControllerPort = 1;
  }

  public static final double DEADBAND = 0.1;
  public static final double maximumSpeed = Units.feetToMeters(4.5);
  public static final TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.HIGH;

  public static class ArmConstants {
    public static final double shoulderOffset = 0;
    public static final double shoulderMin = 0;
    public static final double shoulderMax = 0;
    public static final double shoulderID  = -1;

    public static final double wristOffset = 0;
    public static final double wristID  = -1;

    public static final double telescopeOffset = 0;
    public static final double telescopeMin = 0;
    public static final double telescopeMax = 0;
    public static final double telescopeID  = -1;

    public static final ArmPosition[] positions = {
      new ArmPosition(0, 0, 0, "example")
    };
    
  }
}
