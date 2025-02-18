package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Encoder {
    CANcoder cancoder;
    DutyCycleEncoder dutyEncoder;
    EncoderType type;
    double offset = 0;
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
                return cancoder.getAbsolutePosition().getValueAsDouble()-offset;
            case DutyCycle:
                return dutyEncoder.get()-offset;
            default:
            return-1;
        }
    }
    public void setOffset(double offset){
        this.offset = offset;
    }
    public boolean isConnected(){
        switch (type) {
            case cancoder:
                return cancoder.isConnected();
            case DutyCycle:
                return dutyEncoder.isConnected();
            default:
                return false;
        }
    }
}
