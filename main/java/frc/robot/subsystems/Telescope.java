package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Motor.MotorType;

public class Telescope extends Joint{
    // Constructor to initialize the Joint subsystem
    public Telescope(int jointNum, int motorID, int encoderID, boolean inverted, int defaultSetpointIndex, double[] setpoints, double kP, double kI, double kD,MotorType type) {
        super(jointNum,motorID,encoderID,inverted,defaultSetpointIndex,setpoints,kP,kI,kD,type);
        this.getPID().disableContinuousInput();
    }

    // Method to get the angle in degrees based on encoder position
    @Override
    public double getAngleDegrees() {
        return this.getEncoder().getPosition().getValueAsDouble();
    }

    
}
