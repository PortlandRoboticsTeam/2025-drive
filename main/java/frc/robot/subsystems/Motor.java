package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

public class Motor {
    MotorType type;
    SparkMax sparkMax;
    TalonFX talon;

    public enum MotorType{
        SparkMax,
        talon,
        Ghost // this is for motors that dont exist yet
    }
    public Motor(int motorID, MotorType type) {
        this.type = type;
        switch (type) {
            case SparkMax:
                sparkMax = new SparkMax(motorID, SparkLowLevel.MotorType.kBrushless);
            break;
            case talon:
                talon = new TalonFX(motorID);
            break;
        
            default:
                break;
        }
    }
    public void set(double output) {
        switch (type) {
            case SparkMax:
                sparkMax.set(output);
                break;
            case talon:
                talon.set(output);
                break;
            default:
                break;
        }
    }

}