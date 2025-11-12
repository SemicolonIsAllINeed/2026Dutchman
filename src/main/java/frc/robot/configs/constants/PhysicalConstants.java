package frc.robot.configs.constants;
import edu.wpi.first.math.util.Units;

public class PhysicalConstants {
    public static final class Elevator {
		//all in inches of carriage
		public static final double ROBOT_VOLTAGE = 12.0; 

		//3 is elevator stage count
		public static final double ROTATIONS_TO_INCHES = 5.642 * 3;

		//The maximum inch displacement for every 0.02 sec cycle, has to be less than half of the rotations to inches		
		public static final double MAXIMUM_INCH_CHANGE = 5;
		public static final double GEAR_RATIO = 16;

		public static final double ELEVATOR_LOWER_LIMIT_SWITCH_HEIGHT = 0;
		// public static final double ELEVATOR_UPPER_LIMIT_SWITCH_HEIGHT = -71.8;
		public static final double ELEVATOR_UPPER_LIMIT_SWITCH_HEIGHT = -69.4;

		/*
			to fix offset, move the elevator to its lowest possible position, 
			then subtract the position value from the current offset value
			and make that the new offset
		*/
		public static final double ELEVATOR_OFFSET = -23.11;
		public static final double ELEVATOR_START_HEIGHT = 0.8; //for relative encoder

		public static final double ELEVATOR_REST_HEIGHT = 5.6;

		//coral
		public static final double ELEVATOR_L1_HEIGHT = 0;
		public static final double ELEVATOR_L2_HEIGHT = 27.6;
		public static final double ELEVATOR_L3_HEIGHT = 43.6;
		public static final double ELEVATOR_L4_HEIGHT = 68;

		public static final double ELEVATOR_AUTO_L4_HEIGHT = 69;
		public static final double ELEVATOR_INTAKE_HEIGHT = 17.25;
		public static final double ELEVATOR_AUTO_INTAKE_HEIGHT = 17.25; 

		//algae
		public static final double ELEVATOR_LOW_ALGAE_HEIGHT = 4.4;
		public static final double ELEVATOR_HIGH_ALGAE_HEIGHT = 22.3;
		public static final double ELEVATOR_PROCESSOR_HEIGHT = 0;
		public static final double ELEVATOR_BARGE_HEIGHT = 71.5;
		public static final double ELEVATOR_FLOOR_HEIGHT = 0;
		public static final double ELEVATOR_POP_HEIGHT = 0;
	}
}
