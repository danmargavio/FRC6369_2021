package frc.robot;

public class field {
    private final double length, width;
    private double min_distance;
    private final double[][] obstacle, ball;
    private double[] ball_location = {100, 100};
    public field(){
        length = 0.0;
        width = 0.0;
        min_distance = 0.0;
        obstacle = new double[0][0];
        ball = new double[0][0];
    }
    /**
     * @return the length
     */
    public double getLength() {
        return length;
    }
    /**
     * @return the width
     */
    public double getWidth() {
        return width;
    }
    /**
     * @return the obstacle position
     */
    public double[][] getObstacle() {
        return obstacle;
    }
    /**
     * @param pos = your current position
     * @return the ball position closest to you
     */
    public double[] getBall(double[] pos){
        for(double[] b : ball)
            if(min_distance > distanceToBall(b)){
                min_distance = distanceToBall(b);
                ball_location = b;
            }
        return ball_location;
    }
    /**
     * This function finds out how far away you are from a ball
     * @param ball = location of the ball on the field
     * @return distance to the ball
     */
    public double distanceToBall(double[] ball){
        return Math.sqrt(Math.pow(ball[0] - subSystem.getInstance().field_position(new double[]{DrivetrainSubsystem.getInstance().getGyro().getAngle(), subSystem.getInstance().distance_estimator(Robot.TY)})[0], 2) + Math.pow(ball[1] - subSystem.getInstance().field_position(new double[]{DrivetrainSubsystem.getInstance().getGyro().getAngle(), subSystem.getInstance().distance_estimator(Robot.TY)})[1], 2));
    }
}