// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** An example command that uses an example subsystem. */
public class Tracking_PID extends PIDCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Tracking_PID(DrivetrainSubsystem m_driveSubsystem,
    CameraSubsystem m_CameraSubsystem) {
    super(
      new PIDController(Constants.SpinKp, 
                        Constants.SpinKi, 
                        Constants.SpinKd),
      m_CameraSubsystem::getX, 
      0.0, 
      output -> m_driveSubsystem.driveSpin(output) , 
      m_driveSubsystem);

      SmartDashboard.putNumber("Target", m_CameraSubsystem.getX());

      getController().enableContinuousInput(-90, 90);

      getController().setTolerance(Constants.kTurnToleranceDeg,
      Constants.kTurnToleranceDegPerS);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
}
}
