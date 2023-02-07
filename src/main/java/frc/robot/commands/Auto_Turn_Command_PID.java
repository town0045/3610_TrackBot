// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** An example command that uses an example subsystem. */
public class Auto_Turn_Command_PID extends PIDCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  // private final Double driveSpeed;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Auto_Turn_Command_PID(DrivetrainSubsystem m_driveSubsystem,
                                Double _angle) {
     super(
      new PIDController(Constants.SpinKp_Turn, 
      Constants.SpinKi_Turn, Constants.SpinKd_Turn),
      m_driveSubsystem::getHeading, 
      _angle, 
      output -> m_driveSubsystem.driveSpin(output) , 
      m_driveSubsystem);


      getController().enableContinuousInput(-90, 90);

      getController().setTolerance(Constants.kTurnToleranceDeg_Turn,
      Constants.kTurnToleranceDegPerS_Turn);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
}
}
