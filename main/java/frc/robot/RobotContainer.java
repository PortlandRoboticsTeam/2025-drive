// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.Joint;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Telescope;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  // hardware objects vv
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  Joint shoulder1 = new Joint(0, ArmConstants.shoulder1ID, ArmConstants.shoulderEncoderID, false, 0, null, 0, 0, 0, null);
  Joint shoulder2 = new Joint(0, ArmConstants.shoulder2ID, ArmConstants.shoulderEncoderID, false, 0, null, 0, 0, 0, null);
  Joint wrist     = new Joint(2, ArmConstants.wristID   , ArmConstants.wristEncoderID   , false, 0, null, 0, 0, 0, null);
  Telescope telescope = new Telescope(1, ArmConstants.telescopeID, ArmConstants.telescopeEncoderID, false, 0, null, 0, 0, 0, null);

  // controllers vv
  private final static CommandGenericHID m_driverController =
        new CommandGenericHID(OperatorConstants.driverControllerPort);
  private final static CommandPS4Controller m_helperController =
        new CommandPS4Controller(OperatorConstants.helperControllerPort);

  // variables vv
  private Command driveCommand; 
  private boolean isFieldOriented = true;

  // commands vv
  InstantCommand robotOrientCommand = new InstantCommand(()->{ isFieldOriented = true; });
  InstantCommand fieldOrientCommand = new InstantCommand(()->{ isFieldOriented = false; });
  InstantCommand[] goToPositionCommand = new InstantCommand[ArmConstants.positions.length];
  InstantCommand runArmManualCommand = new InstantCommand(()->{
    shoulder1.setSetpoint(shoulder1.getSetpoint() + MathUtil.applyDeadband(m_helperController.getLeftY (), ArmConstants.manualControlJoystickDeaband));
    shoulder2.setSetpoint(shoulder1.getSetpoint());
    wrist    .setSetpoint(wrist    .getSetpoint() + MathUtil.applyDeadband(m_helperController.getLeftX (), ArmConstants.manualControlJoystickDeaband));
    telescope.setSetpoint(telescope.getSetpoint() + MathUtil.applyDeadband(m_helperController.getRightY(), ArmConstants.manualControlJoystickDeaband));
  });

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      // #keypoints defining the drive command
      driveCommand = swerve.driveCommand(
        () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(1), Constants.DEADBAND),
        () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(0), Constants.DEADBAND),
        () ->  MathUtil.applyDeadband(m_driverController.getRawAxis(2), Constants.DEADBAND), 
        () ->  isFieldOriented
      );
      
      swerve.setDefaultCommand(driveCommand);


      // #keypoints defining all the setpoint commands
      for(int i = 0; i<ArmConstants.positions.length; i++){
        ArmPosition thisArmPosition = ArmConstants.positions[i];
        goToPositionCommand[i] = new InstantCommand(()->{
          wrist    .setSetpoint(thisArmPosition.getWristPos    ());
          shoulder1.setSetpoint(thisArmPosition.getShoulderPos ());
          shoulder2.setSetpoint(thisArmPosition.getShoulderPos ());
          telescope.setSetpoint(thisArmPosition.getTelescopePos());
          SmartDashboard.putString("Arm Setpoint", thisArmPosition.getName());
        });
      }


      // #keypoints Configure the trigger bindings
      configureBindings();
    }
  
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
      m_driverController.button(11).onTrue(swerve.getResetGyro());
      m_driverController.button(2).whileTrue(swerve.vishionDrive());
      m_driverController.button(5).onFalse(fieldOrientCommand);
      m_driverController.button(5).onTrue(robotOrientCommand);

      CommandPS4Controller hc = m_helperController;

      // see the control guide:
      // https://docs.google.com/presentation/d/1Df-ZnbTBpNsHdl3szxJ4QNOpd_V1Ua5YG6Antzs98B0/edit#slide=id.g2af72db80bc_0_5

      // reef scoring:   (1-9)
      hc.cross   ().and(hc.povDown ()).onTrue(goToPositionCommand[1]);
      hc.cross   ().and(hc.povRight()).onTrue(goToPositionCommand[2]);
      hc.cross   ().and(hc.povLeft ()).onTrue(goToPositionCommand[3]);
      hc.circle  ().and(hc.povDown ()).onTrue(goToPositionCommand[4]);
      hc.square  ().and(hc.povRight()).onTrue(goToPositionCommand[5]);
      hc.square  ().and(hc.povLeft ()).onTrue(goToPositionCommand[6]);
      hc.circle  ().and(hc.povUp   ()).onTrue(goToPositionCommand[7]);
      hc.triangle().and(hc.povRight()).onTrue(goToPositionCommand[8]);
      hc.triangle().and(hc.povLeft ()).onTrue(goToPositionCommand[9]);

      // floor pickup:  (12-15)
      hc.povDown().and(hc.L2()).and(hc.circle()).onTrue(goToPositionCommand[12]);
      hc.povDown().and(hc.L2()).and(hc.cross ()).onTrue(goToPositionCommand[13]);
      hc.povUp  ().and(hc.L2()).and(hc.circle()).onTrue(goToPositionCommand[14]);
      hc.povUp  ().and(hc.L2()).and(hc.cross ()).onTrue(goToPositionCommand[15]);

      // other positions:  (16-18)
      hc.L1().and(hc.povUp  ()).and(hc.cross ()).onTrue(goToPositionCommand[16]);
      hc.L1().and(hc.povDown()).and(hc.circle()).onTrue(goToPositionCommand[17]);
      hc.L1().and(hc.povUp  ()).and(hc.circle()).onTrue(goToPositionCommand[18]);
      
      // rest position, zero gyro, and manual arm
      hc.cross().and(hc.R2()).onTrue(goToPositionCommand[0]);
      hc.square().and(hc.R2()).onTrue(swerve.getResetGyro());
      hc.R2().whileTrue(runArmManualCommand);

    }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      // An example command will be run in autonomous
      return swerve.getAutonomousCommand("test");
    }
    
    public SwerveSubsystem getSwerve() {
      return swerve;
    }
    public static CommandGenericHID getController() {
      return m_driverController;
  }
}
