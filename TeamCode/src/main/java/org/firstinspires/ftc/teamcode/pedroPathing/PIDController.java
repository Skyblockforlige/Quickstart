package pedroPathing;

public class PIDController {
    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;
    private long lastTime;

    public PIDController(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.lastTime = System.nanoTime();
    }

    public double update(double target, double current) {
        double error = target - current;

        long now = System.nanoTime();
        double deltaTime = (now - lastTime) / 1e9; // convert ns to seconds
        lastTime = now;

        integralSum += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        return (kP * error) + (kI * integralSum) + (kD * derivative);
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTime = System.nanoTime();
    }
}
