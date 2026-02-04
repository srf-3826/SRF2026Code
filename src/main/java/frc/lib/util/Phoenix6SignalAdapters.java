package frc.lib.util;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.geometry.Rotation2d;

public class Phoenix6SignalAdapters {

    public static class AngleSignal {
        private final StatusSignal<Angle> m_rotationsSignal;
        // Constructor accepts all Phoenix6 motor controller StatusSignal<Angle>
        // objects, as well as CANcoder Absolute <Angle> objects, wrapping them 
        // in a Phoenix6SignalAdapter.AngleSignal type, which provides
        // clean getters for whatevery requested units are needed: Rotations, 
        // Degrees, Radians, or Rotation2d. Note: the range will vary depending on
        // the underlying Signal. 
        public AngleSignal(StatusSignal<Angle> rotationsSignal) {
            this.m_rotationsSignal = rotationsSignal;
        }

        /** Angle in raw motor rotations (-16K to + 16K, or 0 to 1) */
        public double rotations() {
            // The following converts strongly typed StatusSignal<Angle> to Double
            m_rotationsSignal.refresh();
            return m_rotationsSignal.getValueAsDouble();
        }

        /** Angle in degrees, normalized into 0 to 360 range */
        public double degrees() {
            double deg = rotations() * 360.0;
            return ((deg % 360) + ((deg < 0.0) ? 360 : 0.0));
        }

        /** Angle in radians, normalized into range 0 to 2*PI */
        public double radians() {
            double rad = rotations() * 2.0 * Math.PI;
            return (rad % (2 * Math.PI) + ((rad < 0.0) ? (2 * Math.PI) : 0));
        }

        /** WPILib Rotation2d */
        public Rotation2d asRotation2d() {
            return Rotation2d.fromDegrees(degrees());
        }
    }

    public static class DriveSignals {
        private final StatusSignal<Angle> m_rotSignal;
        private final StatusSignal<AngularVelocity> m_velSignal;
        private final double m_rotToMeters;

        // This constructor converts typical StatusSignals associated with a 
        // Drive motor, namely Angle (Position) as rotations, and 
        // AngularVelocity as rotations per second, into a single 
        // Phoenix6SignalAdapter.DriveSignals class whose 
        // methdods will return distance or velocity in WPI friendly units 
        // of meters and meters/sec, as long as created with the correct
        // combined gear and drive wheel size factor which
        // converts rotations to meters (typically just wheel circumference
        // since the motor controller is generally programmed to take gear
        // ratios into account, and reported rotations are actually for 
        // the mechanism's output shaft). This adapter will handle other
        // approaches as long as the final conversion factor is pre-calculated.
        public DriveSignals(StatusSignal<Angle> posRotsSignal,
                            StatusSignal<AngularVelocity> velRpsSignal,
                            double factorRotationsToMeters ) {
            this.m_rotSignal   = posRotsSignal;
            this.m_velSignal   = velRpsSignal;
            this.m_rotToMeters = factorRotationsToMeters;
        }

        /** Position in meters */
        public double positionMeters() {
            m_rotSignal.refresh();
            return this.m_rotSignal.getValueAsDouble() * m_rotToMeters;
        }

        /** Velocity in meters per second */
        public double velocityMps() {
            m_velSignal.refresh();
            return this.m_velSignal.getValueAsDouble() * m_rotToMeters;
        }
    }
}
