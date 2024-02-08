package frc.robot.commands.Vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToObjectCmd extends Command
{

  private final SwerveSubsystem swerveSubsystem;
  private final PIDController   xController;
  private final PIDController   yController;
  // private final PIDController   zController;

  public DriveToObjectCmd(SwerveSubsystem swerveSubsystem)
  {
    this.swerveSubsystem = swerveSubsystem;
    xController = new PIDController(.25, 0.01, 0.0001);
    yController = new PIDController(0.0625, 0.00375, 0.0001);
    // zController = new PIDController(0.0575,0.0, 0.000);
    xController.setTolerance(1);
    yController.setTolerance(1);
    // zController.setTolerance(.5);
    // xController.setSetpoint(1.0);
    // yController.setSetpoint(0.0);
    // zController.setSetpoint(0.0);

    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {

  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    var result = Robot.camObj.getLatestResult();  // Get the latest result from PhotonVision
    boolean hasTargets = result.hasTargets(); // Check if the latest result has any targets.
    PhotonTrackedTarget target = result.getBestTarget();
    
    while (hasTargets == true) {
      double TY = target.getYaw();
      double TX = target.getPitch();

      double translationValx = MathUtil.clamp(xController.calculate(TX, 0.0), -1.0 , 1.0); //* throttle, 2.5 * throttle);
      double translationValy = MathUtil.clamp(yController.calculate(TY, 0.0), -1.0 , 1.0); //* throttle, 2.5 * throttle);
      
      //double translationValz = MathUtil.clamp(zController.calculate(tz, 0.0), -2.0 , 2.0); //* throttle, 2.5 * throttle);

      // SmartDashboard.putString("PhotoVision Target", "True");
      // SmartDashboard.putNumber("PhotonVision Yaw", TY);
      // SmartDashboard.putNumber("PhotonVision Pitch", TX);
      // SmartDashboard.putNumber("TranslationX", translationValX);
      // SmartDashboard.putNumber("TranslationY", translationValY);

      swerveSubsystem.drive(new Translation2d(translationValx, 0.0),
                                              translationValy,
                                              false);
    }
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return xController.atSetpoint() && yController.atSetpoint();
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kBothRumble, 0);
  }
}