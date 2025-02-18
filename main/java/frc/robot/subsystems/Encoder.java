package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Encoder {
    CANcoder cancoder;
    DutyCycleEncoder dutyEncoder;
    EncoderType type;
    public enum EncoderType{
        cancoder,
        DutyCycle
    }
    public Encoder(int id, EncoderType type){
        this.type = type;
        switch (type) {
            case cancoder:
                cancoder = new CANcoder(id);
                break;
            case DutyCycle:
                dutyEncoder = new DutyCycleEncoder(id);
            default:
                break;
        }
    }
    public double getValue(){
        switch (type) {
            case cancoder:
                return cancoder.getAbsolutePosition().getValueAsDouble();
            case DutyCycle:
                return dutyEncoder.get();
            default:
            return-1;
        }
    }
}
