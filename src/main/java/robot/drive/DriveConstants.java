package robot.drive;

public class DriveConstants {
  public static final double WHEEL_RADIUS = 0.08; //Meters
  public static final double CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS;
  public static final double GEARING = 8.0;
} //currently fake values, make sure to have real values for a real robot and yes with wheels.

public static final double POSITION_FACTOR = CIRCUMFERENCE * GEARING;

public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.0;