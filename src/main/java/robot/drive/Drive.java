package robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.Ports;



public class Drive extends SubsystemBase {
    private final DifferentialDriveOdometry odometry;
    private final CANSparkMax leftLeader = new CANSparkMax(Ports.Drive.LEFT_LEADER, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(Ports.Drive.LEFT_FOLLOWER, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(Ports.Drive.RIGHT_LEADER, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(Ports.Drive.RIGHT_FOLLOWER, MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
    private final AnalogGyro gyro = new AnalogGyro(Ports.Drive.GYRO_CHANNEL);
} 

public Drive() {
    for (CANSparkMax spark : List.of(leftLeader, leftFollower, rightLeader, rightFollower)) {
	    spark.restoreFactoryDefaults();
        spark.setIdleMode(IdleMode.kBrake);
    }
    
    rightFollower.follow(rightLeader);
    leftFollower.follow(leftLeader);
    leftLeader.setInverted(true);
    //I hope that I only need to get left leader to inverted so follower also follows the invert.
   //I think that the kBrake and stuff are imported right?
   leftEncoder.setPosition(0);
   rightEncoder.setPosition(0);

   leftEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);
   rightEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);

   leftEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
   rightEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
   //You didn't tell me where to put it, and GPT told me it should be here too
   gyro.reset();

   odometry = new DifferentialDriveOdometry(
    new Rotation2d(), 
    0, 
    0, 
    new Pose2d());
}
  
public void updateOdometry(Rotation2d rotation) {
    odometry.update(rotation, leftEncoder.getPosition(), rightEncoder.getPosition());
  }   

  @Override 
  public void periodic() {
    updateOdometry(gyro.getRotation2d());
  }

  public void resetOdometry(Pose2d pose) {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    odometry.resetPosition(gyro.getRotation2d(), 0, 0, pose);
}//I used chatgpt for the resetOdometry lol, also where are we pulling the methods from anyways

public Pose2d pose() {
    return odometry.getPoseMeters();
  }

  private void drive(double leftSpeed, double rightSpeed) {
    leftLeader.set(leftSpeed);
    rightLeader.set(rightSpeed);
  }
  
  public Command drive(DoubleSupplier vLeft, DoubleSupplier vRight)

  private Command drive(double vLeft, double vRight) {
    return run(() -> drive(vLeft.getAsDouble(), vRight.getAsDouble()));
  }