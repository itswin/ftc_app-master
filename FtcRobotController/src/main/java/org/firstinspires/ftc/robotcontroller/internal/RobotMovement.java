package org.firstinspires.ftc.robotcontroller.internal;

/**
 * Created by Winston on 10/13/2016.
 */

public class RobotMovement
{
    private float mmYDistance;
    private float mmXDistance;
    // Y_AXIS Forward (Pos)/Backward (Neg)
    // X_AXIS Right (Pos)/Left (Neg)

    public RobotMovement(float mmXDistance, float mmYDistance) {
        this.mmXDistance = mmXDistance;
        this.mmYDistance = mmYDistance;
    }

    public float getMmYDistance() { return this.mmYDistance; }

    public float getMmXDistance() { return this.mmXDistance; }

    public boolean isNoMovement()
    {
        return this.mmXDistance == 0 && this.mmYDistance == 0;
    }

    public String toString()
    {
        return String.format("X: %.2f  Y: %.2f", mmXDistance, mmYDistance);
    }
}
