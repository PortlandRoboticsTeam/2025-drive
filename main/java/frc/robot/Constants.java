package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;

public class Constants {

    public static final double MAX_SPEED = 1;
    public static final double LOOP_TIME = 0.1;
    public static final double ROBOT_MASS = 45;
    public static final Matter CHASSIS = new Matter(new Translation3d(), ROBOT_MASS);
    public static final double TURN_CONSTANT = .1;
    public static final double DEADBAND = .1;
    
}
