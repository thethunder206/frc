package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;
  private final ADIS16470_IMU imu = new ADIS16470_IMU();

  public SwerveSubsystem() {
    File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      swerveDrive = new SwerveParser(directory)
          .createSwerveDrive(
            Constants.maxSpeed,
            new Pose2d(
              new Translation2d(1.0, 4.0),
              Rotation2d.fromDegrees(0)
            )
          );
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public void periodic() { }

  @Override
  public void simulationPeriodic() { }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  /** If you ever still want this, leave it; otherwise you can delete it. */
  public void zeroGyro() {
    imu.reset();
  }

  public void driveFieldOriented(ChassisSpeeds speeds) {
    swerveDrive.driveFieldOriented(speeds);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> speedsSupplier) {
    return runOnce(() -> swerveDrive.driveFieldOriented(speedsSupplier.get()));
  }
}



