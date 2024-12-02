package robot;

public final class Ports {
  // TODO: Add and change all ports as needed.
  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }//I think we can delete the operator and use 0, but IDK

public static final class Drive {
  public static final int RIGHT_LEADER = 2;
  public static final int RIGHT_FOLLOWER = 3;
  public static final int LEFT_LEADER = 4;
  public static final int LEFT_FOLLOWER = 5;
  public static final int AnalogGyro = 6;
  }
}