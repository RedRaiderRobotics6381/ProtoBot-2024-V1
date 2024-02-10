// package frc.robot.commands.Vision;
// import java.util.function.Supplier;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonTrackedTarget;

// // import edu.wpi.first.math.MathUtil;
// // import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// // import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
// // import frc.robot.RobotContainer;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// // import edu.wpi.first.wpilibj.XboxController;


// public class DriveToAprilTagPosCmd extends Command
// {
//   //private int visionObject;
//   private int aprilTagID;
//   // private double xOffset;
//   // private double yOffset;
//   // private double omegaOffset;

//   private final SwerveSubsystem swerveSubsystem;
//   private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 1.0);
//   private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 1.0);
//   private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
//   private static final Transform3d TAG_TO_GOAL = new Transform3d(
//                                                                  new Translation3d(Units.inchesToMeters(60), 0, 0),
//                                                                  new Rotation3d(0.0,0.0,Math.PI));
  
//   //public static PhotonCamera camAprTgHigh = new PhotonCamera("camAprTgHigh");
//   private final PhotonCamera camAprTgHigh;
//   private final PhotonCamera camAprTgLow;
//   private final Supplier<Pose2d> poseProvider;
//   private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
//   private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
//   private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

//   private PhotonTrackedTarget lastTargetHigh;
//   private PhotonTrackedTarget lastTargetLow;



  
//   //private double previousPipelineTimestamp = 0;
  
//   //final double TARGET_HEIGHT_METERS = Units.feetToMeters(4.33854166667);
//   //final double TARGET_PITCH_RADIANS = Units.degreesToRadians(0); //angle of the target
  
//   //final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(12);
//   //final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(10); //angle of the camera

//   //final double GOAL_RANGE_METERS = Units.feetToMeters(5); //Distance from the goal to stop at
  
//   // public DriveToAprilTagPosCmd(PhotonCamera photonCamera,
//   //                              SwerveSubsystem swerveSubsystem,
//   //                              Supplier<Pose2d> poseProvider,
//   //                              int visionObject,
//   //                              int aprilTagID,
//   //                              double xOffset,
//   //                              double yOffset,
//   //                              double omegaOffset)
//   // {
//     public DriveToAprilTagPosCmd(PhotonCamera camAprTgHigh, PhotonCamera camAprTgLow, SwerveSubsystem swerveSubsystem, int aprilTagID)
//   {  
//     // each subsystem used by the command must be passed into the
//     // addRequirements() method (which takes a vararg of Subsystem)
//     this.swerveSubsystem = swerveSubsystem;
//     this.poseProvider = swerveSubsystem::getPose;
//     this.aprilTagID = aprilTagID;
//     this.camAprTgHigh = camAprTgHigh;
//     this.camAprTgLow = camAprTgLow;

//     // this.xOffset = xOffset;
//     // this.yOffset = yOffset;
//     // this.omegaOffset = omegaOffset;    

//     // xController = new PIDController(.25, 0.01, 0.0001);
//     // yController = new PIDController(0.0625, 0.00375, 0.0001);
//     // zController = new PIDController(0.0575,0.0, 0.000);

//     // xController.setIZone(0.1); //0.1 meters
//     // yController.setIZone(0.1); //0.1 meters
//     // zController.setIZone(0.5); //0.5 degrees

//     xController.setTolerance(0.2); //0.2 meters
//     yController.setTolerance(0.2); //0.2 meters
//     omegaController.setTolerance(3.0); //3 degrees
//     omegaController.enableContinuousInput(-Math.PI, Math.PI);
    
//     addRequirements(swerveSubsystem); 
        
//   }

//   /**
//    * The initial subroutine of a command.  Called once when the command is initially scheduled.
//    */
//   @Override
//   public void initialize()
//   {
//     //camAprTgHigh.setLED(VisionLEDMode.kDefault);
//     //camAprTgHigh.setPipelineIndex(visionObject);
//     lastTargetHigh = null;
//     lastTargetLow = null;
//     var robotPose = poseProvider.get();
//     omegaController.reset(robotPose.getRotation().getRadians());
//     xController.reset(robotPose.getX());
//     yController.reset(robotPose.getY());
//   }

//   /**
//    * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
//    * until {@link #isFinished()}) returns true.)
//    */
//   @Override
//   public void execute()
//   {
//     var robotPose2d = poseProvider.get();
//     var robotPose = new Pose3d(
//         robotPose2d.getX(),
//         robotPose2d.getY(),
//         0.0,
//         new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

//     var photonResHigh = camAprTgHigh.getLatestResult();
//     if (photonResHigh.hasTargets()) {
//       //Find the tag we want to chase
//       var targetOptHigh = photonResHigh.getTargets().stream()
//       .filter(t -> t.getFiducialId() == aprilTagID)
//       .filter(t -> !t.equals(lastTargetHigh) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
//       .findFirst();
//       if (targetOptHigh.isPresent()) {
//         var targetHigh = targetOptHigh.get();
//         // This is new target data, so recalculate the goal
//         lastTargetHigh = targetHigh;

//         // Transform the robot's pose to find the camera's pose
//         var cameraPose = robotPose
//             .transformBy(new Transform3d(new Translation3d(-.170, -.135, -0.175), new Rotation3d()));

//         // Trasnform the camera's pose to the target's pose
//         var camToTargetHigh = targetHigh.getBestCameraToTarget();
//         var targetPoseHigh = cameraPose.transformBy(camToTargetHigh);

//         // Transform the tag's pose to set our goal
//         var goalPoseHigh = targetPoseHigh.transformBy(TAG_TO_GOAL).toPose2d();

//         // Drive
//         xController.setGoal(goalPoseHigh.getX());
//         yController.setGoal(goalPoseHigh.getY());
//         omegaController.setGoal(goalPoseHigh.getRotation().getRadians());
//       }
//     }

//     if (lastTargetHigh == null) {
//       // No target has been visible
//       swerveSubsystem.lock();
//     } else {
//       // Drive to the target
//       var xSpeed = xController.calculate(robotPose.getX());
//       if (xController.atGoal()) {
//         xSpeed = 0;
//       }

//       var ySpeed = yController.calculate(robotPose.getY());
//       if (yController.atGoal()) {
//         ySpeed = 0;
//       }

//       var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
//       if (omegaController.atGoal()) {
//         omegaSpeed = 0;
//       }

//       swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
//     }
//   }

//   @Override
//   public void end(boolean interrupted) {
//     swerveSubsystem.lock();
//   }

// }
