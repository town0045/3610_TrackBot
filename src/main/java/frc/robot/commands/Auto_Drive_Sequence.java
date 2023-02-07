package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class Auto_Drive_Sequence extends SequentialCommandGroup {
  /**
   * Creates a new ComplexAuto.
   *
   * @param drive The drive subsystem this command will run on
   * @param hatch The hatch subsystem this command will run on
   */
  public Auto_Drive_Sequence(DrivetrainSubsystem m_driveSubsystem) {
    addCommands(
        // Drive forward for the specified time
        new Auto_Drive_Command(m_driveSubsystem, 
                              -1 * Constants.AutoDriveSpeed,
                               1.5 ),
        // Turn the specified degrees
        //new ResetHeading(m_driveSubsystem),
        
        new Auto_Drive_Command(m_driveSubsystem, 
        0.0,
         2.0 ),
        new Auto_Drive_Command(m_driveSubsystem, 
        Constants.AutoDriveSpeed,
         1.5 )
        
        );
  }
}