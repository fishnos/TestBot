package frc.robot.subsystems.drivetrain.swerve;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.*;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Threads;

public class Phoenix6Odometry {
    AtomicBoolean isRunning = new AtomicBoolean(false); //same as a boolean, but thread-safe for multithreading capabilities
    Thread thread;

    private static final int START_THREAD_PRIORITY = 1;
    private final MedianFilter peakRemover = new MedianFilter(3);
    private final LinearFilter lowPass = LinearFilter.movingAverage(50);
    private int successfulDataAcquisitions = 0;
    private int failedDataAcquisitions = 0;
    private double currentTime = 0;
    private volatile double averageLoopTime = 0;
    private int lastThreadPriority = START_THREAD_PRIORITY;
    private int threadPriorityToSet = START_THREAD_PRIORITY;

    // locks allow for thread-safe access to shared resources, meaning only one thread can modify or read the data at a time
    public ReadWriteLock stateLock = new ReentrantReadWriteLock();
    private final Lock signalsLock = new ReentrantLock();

    private BaseStatusSignal[] signals = new BaseStatusSignal[0];
    private double updateFrequency = 100.0;

    private static Phoenix6Odometry instance = null;

    private Phoenix6Odometry() {
        thread = new Thread(this::run);
        thread.setName("Phoenix6OdometryThread");
        thread.setDaemon(true);
    }

    public static Phoenix6Odometry getInstance() {
        if (instance == null) {
            instance = new Phoenix6Odometry();
        }
        return instance;
    }

    public void start() {
        if (!isRunning.getAndSet(true)) {
            thread.start();
        }
    }

    public void stop() {
        isRunning.getAndSet(false);

        try {
            thread.join(); //wait for the thread to die
        } 

        catch (final InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    //adds the signal to the list of signals (locks) and the unlocks
    public void registerSignal(ParentDevice device, BaseStatusSignal signal) {
        signalsLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
            System.arraycopy(signals, 0, newSignals, 0, signals.length);
            newSignals[signals.length] = signal;
            signals = newSignals;
        } finally {
            signalsLock.unlock();
        }
    }

    public void run() {
        BaseStatusSignal.setUpdateFrequencyForAll(updateFrequency, signals); //universal update frequency for all signals
        Threads.setCurrentThreadPriority(true, threadPriorityToSet); //set the thread priority to the desired value

        while (isRunning.get()) {
            StatusCode status = StatusCode.StatusCodeNotInitialized; //make the status not initialized so that it can be set later
            signalsLock.lock();

            try {
                if (signals.length > 0) {
                    status = BaseStatusSignal.waitForAll(2.0 / updateFrequency, signals); //initialize the status to the wait for all signals
                }
            } 
            
            catch (Exception e) {
                e.printStackTrace(); //print the exception stack trace to the console
                Thread.currentThread().interrupt(); //interrupt the current thread if an exception occurs
            } 

            finally {
                signalsLock.unlock();
            }

            if (status != StatusCode.StatusCodeNotInitialized) {
                stateLock.writeLock().lock();

                try {
                    var lastTime = currentTime;
                    currentTime = Utils.getCurrentTimeSeconds();
                    averageLoopTime = lowPass.calculate(peakRemover.calculate(currentTime - lastTime)); //calculate the average loop time using a low pass filter and a peak remover (median filter)
                    if (status.isOK()) { //if the status (the base status signal) is okay, then proceed with the data acquisition
                        successfulDataAcquisitions++;
                    }

                    else {
                        failedDataAcquisitions++;
                    }
                } 
                
                finally {
                    stateLock.writeLock().unlock();
                }
            }

            if (threadPriorityToSet != lastThreadPriority) {
                Threads.setCurrentThreadPriority(true, threadPriorityToSet);
                lastThreadPriority = threadPriorityToSet;
            }
        }
    }

    @SuppressWarnings("unused")
    public boolean odometryIsValid() {
        stateLock.readLock().lock();
        try {
            return successfulDataAcquisitions >= 3; //if there are at least 3 successful data acquisitions, then the odometry is valid
        } finally {
            stateLock.readLock().unlock();
        }
    }

    @SuppressWarnings("unused")
    public double getTime() {
        stateLock.readLock().lock();
        try {
            return averageLoopTime;
        } finally {
            stateLock.readLock().unlock();
        }
    }

    @SuppressWarnings("unused")
    public int getSuccessfulDataAcquisitions() {
        stateLock.readLock().lock();
        try {
            return successfulDataAcquisitions;
        } finally {
            stateLock.readLock().unlock();
        }
    }

    @SuppressWarnings("unused")
    public int getFailedDataAcquisitions() {
        stateLock.readLock().lock();
        try {
            return failedDataAcquisitions;
        } finally {
            stateLock.readLock().unlock();
        }
    }

    @SuppressWarnings("unused")
    public int getThreadPriority() {
        return lastThreadPriority;
    }

    @SuppressWarnings("unused")
    public void setThreadPriority(int priority) {
        threadPriorityToSet = priority;
    }
}