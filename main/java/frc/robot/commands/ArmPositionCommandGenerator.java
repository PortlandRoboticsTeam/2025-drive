package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.Joint;
import frc.robot.subsystems.Telescope;

public class ArmPositionCommandGenerator {
  private ArmPosition pos;
  private Joint shoulder;
  private Telescope telescope;
  private Joint wrist;

  private final ArmPosition tol = ArmConstants.positionCommandCompletionTolerance;

  public ArmPositionCommandGenerator(ArmPosition _pos, Joint s, Telescope t, Joint w){
    pos=_pos; shoulder=s; telescope=t; wrist=w;
  }

  public Command compileCommand(){
    return wrist.getGoToCommand(pos.getWristPos(), tol.getWristPos()).alongWith(
      pos.getShoulderPos() > shoulder.getSetpoint() ?
        shoulder.getGoToCommand(pos.getShoulderPos(), tol.getShoulderPos()).andThen(telescope.getGoToCommand(pos.getTelescopePos(), tol.getTelescopePos())) :
        telescope.getGoToCommand(pos.getTelescopePos(), tol.getTelescopePos()).andThen(shoulder.getGoToCommand(pos.getShoulderPos(), tol.getShoulderPos()))
    );
  }
}
