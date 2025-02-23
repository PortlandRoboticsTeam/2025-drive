package frc.robot.subsystems;

import frc.robot.Constants.GrabberConstants;

public class Grabber {
    private Motor coralMotor = new Motor(GrabberConstants.coralMotorID,GrabberConstants.coralMotorType);
    private Motor algaeMotor = new Motor(GrabberConstants.algaeMotorID,GrabberConstants.algaeMotorType);

    public Grabber(){ }

    public void grab(){
        coralMotor.set(GrabberConstants.coralSpeed);
        algaeMotor.set(GrabberConstants.algaeSpeed);
    }
    public void release(){
        coralMotor.set(-GrabberConstants.coralSpeed);
        algaeMotor.set(-GrabberConstants.algaeSpeed);
    }
    public void stop(){
        coralMotor.set(0);
        algaeMotor.set(0);
    }
    public Motor getCoralMotor(){
        return coralMotor;
    }
    public Motor getAlgaeMotor(){
        return algaeMotor;
    }
}
