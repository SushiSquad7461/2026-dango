package frc.robot.subsystems.Climb;
import static edu.wpi.first.units.Units.*;

import javax.xml.crypto.Data;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.config.ClimbConfig;
import frc.robot.config.ExampleElevatorConfig;

public interface ClimbIO {
	
	@AutoLog
    public static class ClimbData {
		public Distance height = ClimbConfig.ClimbSpecs.MOUNT_OFFSET.getMeasureY();
        public LinearVelocity velocity = MetersPerSecond.of(0);
        public LinearAcceleration accel = MetersPerSecondPerSecond.of(0);
        public double leftCurrentAmps = 0;
        public double rightCurrentAmps = 0;
        public double leftAppliedVolts = 0;
        public double rightAppliedVolts = 0;
    }



	/**
	 * Run the motor at the specified voltage.
	 * 
	 * @param volts
	 */
	public void setVoltage(double volts);

	/**
	 * Enable or disable brake mode on the motor.
	 * 
	 * @param enable
	 */
	public void setBrakeMode(boolean enable);

    /**
	 * Updates the set of loggable inputs.
	 * 
	 * @param data
	 */
	public void updateData();
}