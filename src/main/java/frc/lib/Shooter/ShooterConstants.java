package frc.lib.Shooter;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;

public class ShooterConstants {
    public class SSC {
        public static class Flywheel {  // TODO: This file was pretty much automatically generated (very little oversight, i just clicked "create new constant" a bunch).
                                        // These values need to be verified and/or measured.
            public static final Distance RADIUS = Inches.of(3);
            public static final double GEAR_RATIO = 1/1;

            public static final int MOTOR_ID = 0; // TODO: Put correct ID
            
            public static final double MOTOR_KP = 1;
            public static final double MOTOR_KI = 0;
            public static final double MOTOR_KD = 0;

            // Either enter as a percentage from 0 to 1, or as a direct voltage divided by 12.
            public static final double MOTOR_KS = 0/12;
            public static final double MOTOR_KA = 0/12;
            public static final double MOTOR_KV = 0/12;

            public static final int OUTPUT_MOTOR_LIMIT_FACTOR = 0;

            public static final NeutralModeValue MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;
            public static final InvertedValue MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;

            public static final double OPEN_LOOP_RAMP_PERIOD = 1.0;
            public static final double CLOSED_LOOP_RAMP_PERIOD = 0.0;

            public static final boolean ENABLE_SUPPLY_CURRENT_LIMIT = true;
            public static final double MOTOR_SUPPLY_CURRENT_LIMIT = 35.0;
            public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;
            public static final double STATOR_CURRENT_LIMIT = 80.0;    
        }
        public static class Feed {
            public static final Distance RADIUS = Inches.of(2); 
            public static final double GEAR_RATIO = 1/1;

            public static final int MOTOR_ID = 0; // TODO: Put correct ID
            public static final int ENCODER_ID = 0; // TODO: Put correct ID
        }
    }
} 