package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Winston on 11/8/2016.
 */

public class PIController {
    private double error;
    private double integralErrorX;
    private double integralErrorY;
    private double xKP;
    private double xKI;
    private double yKP;
    private double yKI;
    private double[] motorPowers;

    public PIController(double xKP, double xKI, double yKP, double yKI)
    {
        this.xKP = xKP;
        this.xKI = xKI;
        this.yKP = yKP;
        this.yKI = yKI;
    }

    /**
     * Uses the last location of the robot and the visible beacon to center the robot with the beacon
     * @param lastLocation Last location of the robot
     * @param visibleTarget Name of the target that is seen
     * @return List of movements to make to center the robot with the beacon target
     */
    public RobotMovement processLocation(OpenGLMatrix lastLocation, String visibleTarget)
    {
        float mmXDistance = 0;
        float mmYDistance = 0;

        // TODO Pull robot position back
        if(lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            switch(visibleTarget) {
                case "Wheels":
                    if (translation.getData()[1] <= 1180) {
                        mmYDistance = 1180 - translation.getData()[1];
                    } else if (translation.getData()[1] >= 1220) {
                        mmYDistance = 1220 - translation.getData()[1];
                    }

                    if (translation.getData()[0] >= 50) {
                        mmXDistance = 50 - translation.getData()[0];
                    } else if (translation.getData()[0] <= 10) {
                        mmXDistance = 10 - translation.getData()[0];
                    }
                    break;
                case "Tools":
                    // Old Y: -1180 and -1120
                    if(translation.getData()[0] >= -1250) {
                        mmYDistance = 1250 + translation.getData()[0];
                    } else if(translation.getData()[0] <= -1290) {
                        mmYDistance = 1290 + translation.getData()[0];
                    }

                    // Old Y: 940 and 900
                    if(translation.getData()[1] >= 660) {
                        mmXDistance = 660 - translation.getData()[1];
                    } else if(translation.getData()[1] <= 620) {
                        mmXDistance = 620 - translation.getData()[1];
                    }
                    break;
                case "Legos":
                    // Old Y: 1180 and 1220
                    if (translation.getData()[1] <= 1240) {
                        mmYDistance = 1240 - translation.getData()[1];
                    } else if (translation.getData()[1] >= 1280) {
                        mmYDistance = 1280 - translation.getData()[1];
                    }

                    // Old X: -855 and -895
                    if (translation.getData()[0] >= -1120) {
                        mmXDistance = -(1120 + translation.getData()[0]);
                    } else if (translation.getData()[0] <= -1160) {
                        mmXDistance = -(1160 + translation.getData()[0]);
                    }
                    break;
                case "Gears":
                    // Old Y: -1180 -- -1220
                    if (translation.getData()[0] >= -1250) {
                        mmYDistance = 1250 + translation.getData()[0];
                    } else if (translation.getData()[0] <= -1290) {
                        mmYDistance = 1290 + translation.getData()[0];
                    }

                    // Old X: -260 -- -300
                    if (translation.getData()[1] >= -540) {
                        mmXDistance = -(540 + translation.getData()[1]);
                    } else if (translation.getData()[1] <= -580) {
                        mmXDistance = -(580 + translation.getData()[1]);
                    }
                    break;
            }
        }

        return new RobotMovement(mmXDistance, mmYDistance);
    }

    public TurnMovement processOrientation(OpenGLMatrix lastLocation, String visibleTarget)
    {
        boolean needToTurn = false;
        double turnError = 0;

        // Adjust robot here
        if(visibleTarget.equals("Legos") || visibleTarget.equals("Wheels")) {
            Orientation orientation = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            if(orientation.thirdAngle < 90 && orientation.thirdAngle > 4) {
                turnError = orientation.thirdAngle;
                needToTurn = true;
            } else if(orientation.thirdAngle > -90 && orientation.thirdAngle < -4) {
                turnError = orientation.thirdAngle;
                needToTurn = true;
            } else {
                needToTurn = false;
            }
        } else if(visibleTarget.equals("Gears") || visibleTarget.equals("Tools")) {
            Orientation orientation = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            if(orientation.thirdAngle < 84 && orientation.thirdAngle > 0) {
                turnError = -90 + orientation.thirdAngle;
                needToTurn = true;
            } else if(orientation.thirdAngle > 96 && orientation.thirdAngle < 180) {
                turnError = -90 + orientation.thirdAngle;
                needToTurn = true;
            } else {
                needToTurn = false;
            }
        } else {
            turnError = 0;
            needToTurn = false;
        }

        return new TurnMovement(needToTurn, turnError);
    }

    /**
     *
     * @param mmXDistance Distance to move parallel to the front of the robot
     * @return array with the motor powers for each of the 4 motors on the robot
     *     [0] = frontLeftMotor
     *     [1] = backLeftMotor
     *     [2] = frontRightMotor
     *     [3] = backRightMotor
     */
    public double[] processXMovement(double mmXDistance)
    {
        motorPowers = new double[4];
        error = mmXDistance;
        integralErrorX += mmXDistance;

        double output = xKP * error + xKI * integralErrorX;

        motorPowers[0] = Range.clip(output, -.5, .5);
        motorPowers[1] = Range.clip(-output, -.5, .5);
        motorPowers[2] = Range.clip(-output, -.5, .5);
        motorPowers[3] = Range.clip(output, -.5, .5);

        return motorPowers;
    }

    public double[] processYMovement(double mmYDistance)
    {
        motorPowers = new double[4];
        error = mmYDistance;
        integralErrorY += mmYDistance;

        double output = yKP * error + yKI * integralErrorY;

        motorPowers[0] = Range.clip(output, -.5, .5);
        motorPowers[1] = Range.clip(output, -.5, .5);
        motorPowers[2] = Range.clip(output, -.5, .5);
        motorPowers[3] = Range.clip(output, -.5, .5);

        return motorPowers;
    }

    public void reset()
    {
        integralErrorX = 0;
        integralErrorY = 0;
    }


    public RobotMovement processBeacon(OpenGLMatrix lastLocation, String visibleTarget, String direction)
    {
        float mmXDistance = 0;
        float mmYDistance = 0;

        // TODO Pull robot position back
        if(lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            switch(visibleTarget) {
                case "Wheels":
                    if(direction.equals("left")) {
                        if (translation.getData()[0] >= -30) {
                            mmXDistance = -30 - translation.getData()[0];
                        } else if (translation.getData()[0] <= -70) {
                            mmXDistance = -70 - translation.getData()[0];
                        }
                    } else {
                        if (translation.getData()[0] >= 130) {
                            mmXDistance = 130 - translation.getData()[0];
                        } else if (translation.getData()[0] <= 90) {
                            mmXDistance = 90 - translation.getData()[0];
                        }
                    }
                    break;
                case "Tools":
                    if(direction.equals("left")) {
                        if(translation.getData()[1] >= 580) {
                            mmXDistance = 580 - translation.getData()[1];
                        } else if(translation.getData()[1] <= 540) {
                            mmXDistance = 540 - translation.getData()[1];
                        }
                    } else {
                        if(translation.getData()[1] >= 740) {
                            mmXDistance = 740 - translation.getData()[1];
                        } else if(translation.getData()[1] <= 700) {
                            mmXDistance = 700 - translation.getData()[1];
                        }
                    }
                    break;
                case "Legos":
                    if(direction.equals("left")) {
                        if (translation.getData()[0] >= -1200) {
                            mmXDistance = -(1200 + translation.getData()[0]);
                        } else if (translation.getData()[0] <= -1240) {
                            mmXDistance = -(1240 + translation.getData()[0]);
                        }
                    } else {
                        if (translation.getData()[0] >= -1040) {
                            mmXDistance = -(1040 + translation.getData()[0]);
                        } else if (translation.getData()[0] <= -1080) {
                            mmXDistance = -(1080 + translation.getData()[0]);
                        }
                    }
                    break;
                case "Gears":
                    if(direction.equals("left")) {
                        if (translation.getData()[1] >= -620) {
                            mmXDistance = -(620 + translation.getData()[1]);
                        } else if (translation.getData()[1] <= -660) {
                            mmXDistance = -(660 + translation.getData()[1]);
                        }
                    } else {
                        if (translation.getData()[1] >= -500) {
                            mmXDistance = -(500 + translation.getData()[1]);
                        } else if (translation.getData()[1] <= -540) {
                            mmXDistance = -(540 + translation.getData()[1]);
                        }
                    }
            }
        }

        return new RobotMovement(mmXDistance, mmYDistance);
    }
}
