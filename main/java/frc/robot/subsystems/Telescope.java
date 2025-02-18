package frc.robot.subsystems;

import frc.robot.subsystems.Encoder.EncoderType;
import frc.robot.subsystems.Motor.MotorType;
import frc.robot.subsystems.Joint;

public class Telescope extends Joint{
    // Constructor to initialize the Joint subsystem
    public Telescope(int jointNum, int motorID, int encoderID, boolean inverted, int defaultSetpointIndex, double[] setpoints, double kP, double kI, double kD,MotorType type, EncoderType eType) {
        super(jointNum,motorID,encoderID,inverted,defaultSetpointIndex,setpoints,kP,kI,kD,type,eType);
        this.getPID().disableContinuousInput();
    }

    // Method to get the angle in degrees based on encoder position
    @Override
    public double getAngleDegrees() {
        return this.getEncoder().getValue();
    }
}
