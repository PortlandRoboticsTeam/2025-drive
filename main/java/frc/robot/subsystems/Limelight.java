package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
  NetworkTableInstance tables = NetworkTableInstance.getDefault();
  NetworkTable limeInfo = tables.getTable("limelight");


  public double getx() {
    double p = .06;
    return limeInfo.getEntry("ty").getDouble(0)*p;
  }

  public double getAngle() {
    double p = .01;
    return -limeInfo.getEntry("tx").getDouble(0)*p;
  }

  @Override
  public void periodic() {
      // TODO Auto-generated method stub
      super.periodic();
      SmartDashboard.putNumber("vis out", limeInfo.getEntry("tx").getDouble(0));
  }
}
