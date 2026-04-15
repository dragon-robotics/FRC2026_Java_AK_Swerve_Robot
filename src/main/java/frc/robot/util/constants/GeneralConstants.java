package frc.robot.util.constants;

import edu.wpi.first.wpilibj.RobotBase;

public final class GeneralConstants {

  // Robot mode
  public static final RobotMode CURRENT_MODE = RobotBase.isReal() ? RobotMode.COMP : RobotMode.SIM;

  public static enum RobotMode {
    /** Running on test mode */
    TEST,
    /** Running on competition mode */
    COMP,
    /** Running on simulation mode */
    SIM
  }

  public static boolean disableHAL = false;
}
