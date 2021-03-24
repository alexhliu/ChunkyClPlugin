package chunkycl;

import org.apache.commons.math3.util.FastMath;
import se.llbit.chunky.renderer.scene.Scene;
import se.llbit.log.Log;

import java.util.Arrays;

public class Finalizer {
    public final int MIN_SLEEP = 50;
    public final int MAX_SLEEP = 100;

    protected RenderManagerCl renderer;
    private FinalizeWorker[] threads;

    protected final Object stopMonitor = new Object();
    protected final Object startMonitor = new Object();
    protected final Object completeMonitor = new Object();
    protected volatile boolean stopping = false;

    public Finalizer(RenderManagerCl renderer) {
        this.renderer = renderer;

        threads = new FinalizeWorker[renderer.getNumThreads()];
        for (int i = 0; i < threads.length; i++) {
            threads[i] = new FinalizeWorker(i, this);
            threads[i].setDaemon(true);
            threads[i].start();
        }
    }

    /** Attempt to force stop all render threads. Should be relatively quick. */
    public void stopNow() throws InterruptedException {
        stopping = true;
        synchronized (stopMonitor) {
            stopMonitor.notifyAll();
        }

        join();

        stopping = false;
    }

    /** Wait for all threads to finish. */
    public void join() throws InterruptedException {
        synchronized (completeMonitor) {
            while (isFinalizing()) {
                completeMonitor.wait(50);
            }
        }
    }

    /** Finalize and wait for all threads to finish */
    public void finalizeOnce() throws InterruptedException {
        stopNow();
        synchronized (startMonitor) {
            startMonitor.notifyAll();
        }
        while (!finalized());
        join();
    }

    /** Start finalization process. */
    public void finalizeSoon() {
        if (isFinalizing()) {
            return;
        }

        synchronized (startMonitor) {
            startMonitor.notifyAll();
        }
    }

    /** Check if threads are currently finalizing (false = done). */
    public boolean isFinalizing() {
        return Arrays.stream(threads).anyMatch(worker -> worker.finalizing);
    }

    /** Check if all the threads have started finalizing. Resets flag if true. */
    private boolean finalized() {
        if (Arrays.stream(threads).allMatch(worker -> worker.finalized)) {
            Arrays.stream(threads).forEach(worker -> worker.finalized = false);
            return true;
        }
        return false;
    }

    private class FinalizeWorker extends Thread {
        private final int id;
        private final Finalizer manager;
        protected volatile boolean finalizing = false;
        protected volatile boolean finalized = false;

        public FinalizeWorker(int id, Finalizer finalizer) {
            super("Finalize Worker " + id);

            this.id = id;
            this.manager = finalizer;
        }

        @Override
        public void run() {
            try {
                while (!isInterrupted()) {
                    // Wait for notification to start finalizing pixels
                    synchronized (manager.startMonitor) {
                        manager.startMonitor.wait();
                    }
                    finalizing = true;
                    finalized = true;

                    try {
                        this.finalize(manager.renderer.getBufferedScene(), manager.threads.length, manager.renderer.getCPULoad());
                    } catch (ArrayIndexOutOfBoundsException e) {
                        // Resize?
                    }

                    // Notify done finalizing
                    finalizing = false;
                    synchronized (manager.completeMonitor) {
                        manager.completeMonitor.notifyAll();
                    }
                }
            } catch (InterruptedException e) {
                // Interrupted.
            } catch (Throwable e) {
                Log.error("Error worker " + id + " crashed with an uncaught exception.", e);
            }
        }

        private void finalize(Scene scene, int threads, int cpuLoad) throws InterruptedException, ArrayIndexOutOfBoundsException {
            int width = scene.canvasWidth();
            int height = scene.canvasHeight();

            long jobStart = System.nanoTime();

            // Finalize pixel if (x+y) % threads == id
            for (int i = 0; i < width; i++) {
                for (int j = 0; j < height; j++) {
                    if ((i + j) % threads == id) {
                        scene.finalizePixel(i, j);

                        // Sleep to manage cpu usage
                        if (cpuLoad < 100) {
                            double jobTime = System.nanoTime() - jobStart;
                            jobTime /= 1000000.0;
                            double sleepTime = jobTime / (cpuLoad / 100.0) - jobTime;

                            if (sleepTime > MIN_SLEEP) {
                                synchronized (manager.stopMonitor) {
                                    manager.stopMonitor.wait((long) FastMath.min(sleepTime, MAX_SLEEP));
                                }
                                jobStart = System.nanoTime();
                            }
                        }

                        if (manager.stopping) {
                            return;
                        }
                    }
                }
            }
        }
    }
}
