package frc.robot.utilities;

/**
 * A filter to achieve smooth rate of change to a desired input speed through the equation: <br>
 * </br>
 * output = (alpha*input) + ((1-alpha)*oldspeed) <br>
 * </br>
 * Where alpha is a tunable coefficient.
 *
 * @author Adam N.
 */
public class AlphaFilter implements Filter {

  private double mAlpha;
  private double mFilteredSpeed;

  /**
   * @param The desired alpha coefficients
   */
  public AlphaFilter(final double alpha) {
    mAlpha = alpha;
  }

  /**
   * Calculates the correct output based on a desired value.
   *
   * @param input The desired value to reach
   * @return The calculated output based on previous inputs.
   */
  public double calculate(double input) {
    
    if(input >= 0 && mFilteredSpeed < 0.0) {
      reset();
    } else if (input < 0.0 && mFilteredSpeed >= 0.0) {
      reset();
    }
    mFilteredSpeed = (mAlpha * input) + ((1 - mAlpha) * mFilteredSpeed);
    return mFilteredSpeed;
  }

  /**
   * set the alpha gain forward.
   *
   * @param alpha The alpha gain forward
   */
  public void setAlpha(double alpha) {
    mAlpha = alpha;
  }

  public double getValue() {
    return mFilteredSpeed;
  }


  @Override
  public void reset() {
    mFilteredSpeed = 0.0;
  }
  public void resetWithAngle(double angle) {
    mFilteredSpeed = angle;
  }
}