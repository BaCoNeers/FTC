package org.firstinspires.ftc.teamcode;
/**
 * Created by Simon on 5/12/2017.
 */



public class JewelDrop{


    public void Jewel(boolean toggle, Configuration robot){
        robot.drop.setPosition(1);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
            }
        if (robot.drop.getPosition() == 1 && (robot.Colour.blue() > 0 || robot.Colour.red() > 0)) {
            if (toggle) {
                if (robot.Colour.blue() > robot.Colour.red()) {
                    robot.push.setPosition(1);
                } else {
                    robot.push.setPosition(0);
                }
            } else {
                if (robot.Colour.red() > robot.Colour.blue()) {
                    robot.push.setPosition(1);
                } else {
                    robot.push.setPosition(0);
                }
            }
        }
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.drop.setPosition(0.5);
        robot.push.setPosition(0.45);

    }
}
