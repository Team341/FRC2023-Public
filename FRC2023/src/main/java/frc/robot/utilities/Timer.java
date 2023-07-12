package frc.robot.utilities;

public class Timer implements Runnable {

    double seconds;
    TimerUser user;
    boolean loop = true;
    boolean stop = false;
    Thread thread;
    
    public Timer(double seconds, TimerUser user) {
        this.seconds = seconds;
        this.user = user;
    }
    
    public Timer(double seconds, boolean loop, TimerUser user) {
        this.seconds = seconds;
        this.loop = loop;
        this.user = user;
    }
    
    /**
     * Starts the Timer.
     * @return This timer. To be used for method chaining.
     */
    public Timer start() {
        thread = new Thread(this);
        thread.start();
        return this;
    }
    
    /**
     * The loop that makes this timer work. Don't call this.
     */
    @Override
    public void run() {
        while (true) {
            try {
                if (stop) break;
                Thread.sleep((long) seconds * 1000);
                user.timer();
                if (!loop) break;
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        thread = null;
        user.timerStop();
    }

    /**
     * Stops this timer.
     */
    public void stop() {
        stop = true;
    }   
}