package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  // Driver controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Direct-angle swerve input stream: translation via left stick, module heading via right stick
  private final SwerveInputStream driveDirectAngle = SwerveInputStream
    .of(
      drivebase.getSwerveDrive(),
      () -> -m_driverController.getLeftY(),      // forward/back (invert so push up = +)
      () -> -m_driverController.getLeftX()       // strafe (invert so push right = +)
    )
    .withControllerHeadingAxis(
      m_driverController::getRightX,              // heading X (left/right)
      () -> -m_driverController.getRightY()       // heading Y (forward/back)
    )
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

  // Command wrapping the direct-angle stream
  private final Command driveFieldOrientedDirectAngle =
      drivebase.driveFieldOriented(driveDirectAngle);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Use the direct-angle swerve command by default
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  /**
   * Expose the driver controller, if needed elsewhere (e.g., for teleopInit in Robot.java).
   */
  public CommandXboxController getDriverController() {
    return m_driverController;
  }

    public SwerveSubsystem getDrivebase()
  {
    return drivebase;
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for xbox/ps4
   * controllers or flight joysticks.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}



