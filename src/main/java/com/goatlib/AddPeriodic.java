package com.goatlib;

/**
 Wrapper for TimeRobot.addPeriodic() to pass for other classes usage

 Example Usage:

 new RobotContainer(this::addPeriodic);

 RobotContainer(AddPeriodic addPeriodic) {
 addPeriodic.accept( ()->System.out.println("printing"), 1., 0.);
 }

 */
@FunctionalInterface
public interface AddPeriodic {
    /**
     * @param callback - what to run periodically
     * @param periodSeconds - time between runs
     * @param offsetSeconds - offset from standard loop start time
     */
    void accept(Runnable callback, double periodSeconds , double offsetSeconds);
}
