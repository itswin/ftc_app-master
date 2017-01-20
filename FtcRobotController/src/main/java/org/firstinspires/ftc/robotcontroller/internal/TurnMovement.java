package org.firstinspires.ftc.robotcontroller.internal;

/**
 * Created by Winston on 11/11/2016.
 */

public class TurnMovement {
    private boolean needToTurn;
    private double turnError;

    public TurnMovement(boolean needToTurn, double turnError)
    {
        this.needToTurn = needToTurn;
        this.turnError = turnError;
    }

    public boolean getNeedToTurn() { return needToTurn; }

    public double getTurnError() { return turnError; }

    public String toString()
    {
        return needToTurn + ", " + turnError;
    }
}
