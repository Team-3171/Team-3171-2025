package frc.team3171;

/**
 * @author Mark Ebert
 */
public class HelperFunctions {

	/**
	 * This function returns a value adjusted for the inputed deadzone.
	 * 
	 * @param deadzoneRange
	 *            The range that will be used when adjusting the inputed
	 *            value.
	 * @param currentValue
	 *            The original unaltered value that should range from -1.0
	 *            to 1.0.
	 * @return The value adjusted for the given deadzone range.
	 */
	public static double Deadzone(double deadzoneRange, final double currentValue) {
		deadzoneRange = Math.abs(deadzoneRange);
		return (currentValue <= -deadzoneRange || currentValue >= deadzoneRange ? currentValue : 0);
	}

	/**
	 * This function returns a value adjusted for the inputed deadzone and maps it
	 * to the full range.
	 * 
	 * @param deadzoneRange
	 *            The range that will be used when adjusting the inputed
	 *            value.
	 * @param currentValue
	 *            The original unaltered value that should range from -1.0
	 *            to 1.0.
	 * @return The value adjusted for the given deadzone range.
	 */
	public static double Deadzone_With_Map(double deadzoneRange, final double currentValue) {
		deadzoneRange = Math.abs(deadzoneRange);
		if (currentValue <= -deadzoneRange || currentValue >= deadzoneRange) {
			if (currentValue > 0) {
				return Map(currentValue, deadzoneRange, 1.0, 0.0, 1.0);
			}
			return Map(currentValue, -1.0, -deadzoneRange, -1.0, 0.0);
		}
		return 0;
	}

	/**
	 * This function returns a value adjusted for the inputed deadzone and maps it
	 * to the full range.
	 * 
	 * @param deadzoneRange
	 *            The range that will be used when adjusting the inputed
	 *            value.
	 * @param currentValue
	 *            The original unaltered value that should range from -1.0
	 *            to 1.0.
	 * @return The value adjusted for the given deadzone range.
	 */
	public static double Deadzone_With_Map(double deadzoneRange, final double currentValue, final double out_min, final double out_max) {
		deadzoneRange = Math.abs(deadzoneRange);
		if (currentValue <= -deadzoneRange || currentValue >= deadzoneRange) {
			if (currentValue > 0) {
				return Map(currentValue, deadzoneRange, 1.0, 0.0, out_max);
			}
			return Map(currentValue, -1.0, -deadzoneRange, out_min, 0.0);
		}
		return 0;
	}

	/**
	 * This function returns a value adjusted for the inputed deadzone and maps it
	 * to the full range.
	 * 
	 * @param deadzoneRange
	 *            The range that will be used when adjusting the inputed
	 *            value.
	 * @param currentValuse
	 *            An array of the original unaltered values that should
	 *            range from -1.0 to 1.0.
	 * @return An array, in the same order as they were given, of the values
	 *         adjusted for the given deadzone range.
	 */
	public static double[] Deadzone_With_Map(final double deadzoneRange, final double currentValue,
			final double... currentValues) {
		final double[] newValues = new double[currentValues.length + 1];
		newValues[0] = Deadzone_With_Map(deadzoneRange, currentValue);
		for (int i = 0; i < currentValues.length; i++) {
			newValues[i + 1] = Deadzone_With_Map(deadzoneRange, currentValues[i]);
		}
		return newValues;
	}

	/**
	 * Maps the value from the first range to the second range.
	 * 
	 * @param x:
	 *            the number to map.
	 * @param in_min:
	 *            the lower bound of the value's current range.
	 * @param in_max:
	 *            the upper bound of the value's current range.
	 * @param out_min:
	 *            the lower bound of the value's target range.
	 * @param out_max:
	 *            the upper bound of the value's target range.
	 */
	public static int Map(int x, int in_min, int in_max, int out_min, int out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	/**
	 * Maps the value from the first range to the second range.
	 * 
	 * @param x:
	 *            the number to map.
	 * @param in_min:
	 *            the lower bound of the value's current range.
	 * @param in_max:
	 *            the upper bound of the value's current range.
	 * @param out_min:
	 *            the lower bound of the value's target range.
	 * @param out_max:
	 *            the upper bound of the value's target range.
	 */
	public static double Map(double x, double in_min, double in_max, double out_min, double out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	/**
	 * Determines if the currentValue is within range of the desiredValue by the
	 * given percentage.
	 * 
	 * @param currentValue
	 *            The current value to compare.
	 * @param desiredValue
	 *            The desired value to compare the current value to.
	 * @param errorPercentage
	 *            The desired percent error margin allowed for the
	 *            calculation.
	 * @return true if the currentValue is within the desiredValue by the given
	 *         percent margin, false otherwise.
	 */
	public static boolean Within_Percent_Error(final double currentValue, final double desiredValue,
			final double errorPercentage) {
		final double acceptableError = Math.abs(desiredValue) * Math.abs(errorPercentage);
		return (currentValue >= (desiredValue - acceptableError)) && (currentValue <= (desiredValue + acceptableError));
	}

	/**
	 * Turns the inputed value into one that is more useful when dealing with non-continuous ranges.
	 * 
	 * @param value
	 *            The value to normalize.
	 * 
	 * @param minValue
	 *            The minimum value of the non-continuous range.
	 * 
	 * @param maxValue
	 *            The maximum value of the non-continuous range.
	 * 
	 * @return Returns the given value in a normalized format, using the provided range.
	 */
	public static double Normalize_Value(final double value, final double minValue, final double maxValue) {
		final double maxRange = Math.abs(maxValue - minValue);
		final double temp = value % maxRange;
		return (temp < minValue ? temp + maxRange : temp > maxValue ? temp - maxRange : temp);
	}

	/**
	 * Turns a gyros raw values into angles that are more useful (-180.0 degrees to 180.0 degrees), where 0 is the front and
	 * -180.0 is 180 degrees counterclockwise from 0 and 180.0 is 180 degrees clockwise from 0.
	 * 
	 * @param gyroValue
	 *            The gyro value to normalize.
	 * 
	 * @return Returns the Gyro's angle in a normalized format, from -180 degrees to 180 degrees.
	 */
	public static double Normalize_Gryo_Value(final double gyroValue) {
		return Normalize_Value(gyroValue, -180, 180);
	}

	/**
	 * Returns the shortest displacement in the best direction from the currentValue to the desiredValue. This function will
	 * normalize (scale the values from the provided minValue and maxValue) both the currentValue and the desiredValue and
	 * will return the displacement as a normalized value as well.
	 * 
	 * @param currentValue
	 *            The current value.
	 * @param desiredValue
	 *            The desired value.
	 * @return The shortest displacement representing the best direction to reach the desired value.
	 */
	public static double Get_Displacement(final double currentValue, final double desiredValue, double minValue, double maxValue) {
		final double maxRange = Math.abs(maxValue - minValue);
		minValue = -maxRange / 2;
		maxValue = maxRange / 2;
		final double displacement = Normalize_Value(desiredValue, minValue, maxValue) - Normalize_Value(currentValue, minValue, maxValue);
		return (displacement < minValue ? displacement + maxRange : displacement > maxValue ? displacement - maxRange : displacement);
	}

	/**
	 * Returns the shortest gyro displacement in the best direction from the
	 * currentValue to the desiredValue. This function will normalize, scale the
	 * values from -180.0 degrees to 180.0 degrees, both the currentValue and the
	 * desiredValue and will return the displacement as a normalized value as well.
	 * 
	 * @param currentValue
	 *            The current gyro value in degrees.
	 * @param desiredValue
	 *            The desired gyro value in dgerees.
	 * @return The shortest gyro displacement representing the best direction to
	 *         reach the desired value with the least amount of turning.
	 */
	public static double Get_Gyro_Displacement(final double currentValue, final double desiredValue) {
		final double displacement = Normalize_Gryo_Value(desiredValue) - Normalize_Gryo_Value(currentValue);
		return (displacement < -180 ? displacement + 360 : displacement > 180 ? displacement - 360 : displacement);
	}

	public static double[] Vector_To_Component(final double angle, final double magnitude) {
		final double xComponent = magnitude * Math.cos(Math.toRadians(angle));
		final double yComponent = magnitude * Math.sin(Math.toRadians(angle));
		return new double[] { xComponent, yComponent };
	}

	public static double[] Component_To_Vector(final double xComponent, final double yComponent) {
		final double angle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(yComponent, xComponent)));
		final double magnitude = Math.sqrt(Math.pow(yComponent, 2) + Math.pow(xComponent, 2));
		return new double[] { angle, magnitude };
	}

	public static double[] Add_Two_Components(final double[] componentOne, final double[] componentTwo) {
		return new double[] { (componentOne[0] + componentTwo[0]), (componentOne[1] + componentTwo[1]) };
	}

	public static double[] Add_Two_Vectors(final double[] vectorOne, final double[] vectorTwo) {
		final double[] componentOne = Vector_To_Component(vectorOne[0], vectorOne[1]);
		final double[] componentTwo = Vector_To_Component(vectorTwo[0], vectorTwo[1]);
		final double[] addedComponents = Add_Two_Components(componentOne, componentTwo);
		return Component_To_Vector(addedComponents[0], addedComponents[1]);
	}

	public static double[] Return_Vector_With_Largest_Magnitude(final double[] vector, final double[]... vectors) {
		double[] maxVector = vector;
		for (double[] x : vectors) {
			if (x[1] > maxVector[1]) {
				maxVector = x;
			}
		}
		return maxVector;
	}

}