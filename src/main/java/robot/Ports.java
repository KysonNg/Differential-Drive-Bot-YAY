package robot;

public final class Ports {
  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 2;
  }//I think we can delete the operator and use 0, but IDK

public static final class Drive {
  public static final int RIGHT_LEADER = 3;
  public static final int RIGHT_FOLLOWER = 4;
  public static final int LEFT_LEADER = 5;
  public static final int LEFT_FOLLOWER = 6;
  public static final int GYRO_CHANNEL = 1;
  }
}