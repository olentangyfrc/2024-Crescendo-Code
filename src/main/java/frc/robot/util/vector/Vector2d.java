package frc.robot.util.vector;

import edu.wpi.first.math.geometry.Translation2d;

public class Vector2d {
    private double x;
    private double y;
    
    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector2d fromPolar(double radians, double magnitude) {
        return new Vector2d(magnitude * Math.cos(radians), magnitude * Math.sin(radians));
    }

    public Vector2d() {
        this(0, 0);
    }

    public Vector2d(Translation2d translation) {
        this(translation.getX(), translation.getY());
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double dot(Vector2d other) {
        return x * other.x + y * other.y;
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public double radians() {
        return Math.atan2(y, x);
    }

    public Vector2d times(double scalar) {
        return new Vector2d(x * scalar, y * scalar);
    }

    public Vector2d plus(Vector2d other) {
        return new Vector2d(x + other.x, y + other.y);
    }

    public Vector2d minus(Vector2d other) {
        return new Vector2d(x - other.x, y - other.y);
    }

    public Vector2d inverse() {
        return new Vector2d(-y, x);
    }

    public Translation2d getTranslation2d() {
        return new Translation2d(x, y);
    }

    public double getAngleBetween(Vector2d other){
        return Math.acos(this.dot(other)/(this.magnitude()*other.magnitude()));
    }

    @Override
    public String toString() {
        return "Vector[" + x + ", " + y + "]";
    }
}
