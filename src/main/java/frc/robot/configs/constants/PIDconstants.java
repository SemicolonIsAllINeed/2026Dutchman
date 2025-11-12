package frc.robot.configs.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class PIDconstants {
    public final class Elevator {
		public static final double POS_KP = 0.08;
		public static final double POS_KI = 0;
		public static final double POS_KD = 0.003;
		public static final Constraints POS_CONSTRAINTS = new Constraints(80, 300);
		public static final Constraints POS_TELEOP_CONSTRAINTS = new Constraints(87, 360);
		public static final double SLOW_MAX_ELEVATOR_SPEED = 40;
		public static final double MEDIUM_MAX_ELEVATOR_SPEED = 70;

		// public static final double POS_POS_TOLERANCE = 0.05;
		// public static final double POS_VEL_TOLERANCE = 0;

		public static final double POS_KS = 0;
		public static final double POS_KG = -0.05;
		public static final double POS_KV = 0;
		public static final double POS_KA = 0;
  }
}
