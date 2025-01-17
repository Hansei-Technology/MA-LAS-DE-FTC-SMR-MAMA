package htech.mechanism.hang;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import htech.config.PositionsPTO;
import htech.config.Servos;

public class PTO {

    Servo servoRight, servoLeft;

    public enum STATES{
        COUPLED,
        IDLE,
        NULL
    }

    public STATES ptoState = STATES.NULL;

    public PTO(HardwareMap map){
        servoRight = map.get(Servo.class, Servos.ptoRightServo);
        servoLeft = map.get(Servo.class, Servos.ptoLeftServo);
    }

    public void initPTO(){
        servoRight.setPosition(PositionsPTO.closedRight);
        servoLeft.setPosition(PositionsPTO.closedLeft);
        ptoState = STATES.IDLE;
    }

    public void couple(){
        servoLeft.setPosition(PositionsPTO.openedLeft);
        servoLeft.setPosition(PositionsPTO.openedRight);
        ptoState = STATES.COUPLED;
    }

    public void resetRight(){
        servoRight.setPosition(PositionsPTO.closedRight);
    }

    public void resetLeft(){
        servoLeft.setPosition(PositionsPTO.closedLeft);
    }
}
