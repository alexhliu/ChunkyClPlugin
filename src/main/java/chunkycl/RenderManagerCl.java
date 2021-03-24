package chunkycl;

import se.llbit.chunky.PersistentSettings;
import se.llbit.chunky.main.Chunky;
import se.llbit.chunky.renderer.*;
import se.llbit.chunky.renderer.scene.Camera;
import se.llbit.chunky.renderer.scene.Scene;
import se.llbit.chunky.resources.BitmapImage;
import se.llbit.log.Log;
import se.llbit.math.Ray;
import se.llbit.math.Vector2;
import se.llbit.math.Vector3;
import se.llbit.math.Vector3i;
import se.llbit.util.TaskTracker;

import java.util.*;
import java.util.concurrent.ForkJoinTask;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.stream.IntStream;

public class RenderManagerCl extends Thread implements Renderer {
    private static final Repaintable EMPTY_CANVAS = () -> {};
    private Repaintable canvas = EMPTY_CANVAS;

    private Finalizer finalizer;
    private boolean shouldFinalize = true;
    private final Scene bufferedScene;
    private final boolean headless;
    private int numThreads;

    private RenderMode mode = RenderMode.PREVIEW;

    private SnapshotControl snapshotControl = SnapshotControl.DEFAULT;

    private RenderContext context;

    private Random random;


    private int cpuLoad;
    private SceneProvider sceneProvider;

    private BiConsumer<Long, Integer> renderCompleteListener;
    private BiConsumer<Scene, Integer> frameCompleteListener;

    private Collection<RenderStatusListener> renderListeners = new ArrayList<>();
    private Collection<SceneStatusListener> sceneListeners = new ArrayList<>();

    private TaskTracker.Task renderTask;

    private int drawDepth = 256;
    private boolean drawEntities = true;

    public static final GpuRayTracer intersectCl = GpuRayTracer.getTracer();

    public RenderManagerCl(RenderContext context, boolean headless) {
        super("Render Manager");

        this.setPriority(Thread.MAX_PRIORITY);

        numThreads = context.numRenderThreads();
        cpuLoad = PersistentSettings.getCPULoad();
        finalizer = new Finalizer(this);

        this.context = context;

        this.headless = headless;
        bufferedScene = context.getChunky().getSceneFactory().newScene();

        random = new Random(System.currentTimeMillis());
    }

    public void setDrawDepth(int drawDepth) {
        this.drawDepth = drawDepth;
    }

    public void setDrawEntities(boolean drawEntities) {
        this.drawEntities = drawEntities;
    }

    public int getNumThreads() {
        return numThreads;
    }

    @Override public void setSceneProvider(SceneProvider sceneProvider) {
        this.sceneProvider = sceneProvider;
    }

    @Override public void setCanvas(Repaintable canvas) {
        this.canvas = canvas;
    }

    @Override public void setCPULoad(int loadPercent) {
        this.cpuLoad = loadPercent;
    }

    public int getCPULoad() {
        return cpuLoad;
    }

    @Override public void setOnRenderCompleted(BiConsumer<Long, Integer> listener) {
        renderCompleteListener = listener;
    }

    @Override public void setOnFrameCompleted(BiConsumer<Scene, Integer> listener) {
        frameCompleteListener = listener;
    }

    @Override public void setSnapshotControl(SnapshotControl callback) {
        snapshotControl = callback;
    }

    @Override public void setRenderTask(TaskTracker.Task task) {
        renderTask = task;
    }

    @Override public synchronized void addRenderListener(RenderStatusListener listener) {
        renderListeners.add(listener);
    }

    @Override public void removeRenderListener(RenderStatusListener listener) {
        renderListeners.remove(listener);
    }

    @Override public synchronized void addSceneStatusListener(SceneStatusListener listener) {
        sceneListeners.add(listener);
    }

    @Override public void removeSceneStatusListener(SceneStatusListener listener) {
        sceneListeners.remove(listener);
    }

    @Override public void withBufferedImage(Consumer<BitmapImage> consumer) {
        bufferedScene.withBufferedImage(consumer);
    }

    @Override public void withSampleBufferProtected(SampleBufferConsumer consumer) {
        synchronized (bufferedScene) {
            consumer.accept(bufferedScene.getSampleBuffer(), bufferedScene.width, bufferedScene.height);
        }
    }

    @Override public RenderStatus getRenderStatus() {
        RenderStatus status;
        synchronized (bufferedScene) {
            status = new RenderStatus(bufferedScene.renderTime, bufferedScene.spp);
        }
        return status;
    }

    @Override public void shutdown() {
        interrupt();
    }

    public Scene getBufferedScene() {
        return bufferedScene;
    }

    private void updateRenderState(Scene scene) {
        shouldFinalize = scene.shouldFinalizeBuffer();
        if (mode != scene.getMode()) {
            mode = scene.getMode();
            renderListeners.forEach(listener -> listener.renderStateChanged(mode));
        }
    }

    private synchronized void sendSceneStatus(String status) {
        for (SceneStatusListener listener : sceneListeners) {
            listener.sceneStatus(status);
        }
    }

    private float[] generateCameraRays() {
        // Generate camera starting rays
        int width = bufferedScene.canvasWidth();
        int height = bufferedScene.canvasHeight();

        double halfWidth = width / (2.0 * height);
        double invHeight = 1.0 / height;

        float[] rayDirs = new float[width * height * 3];

        Camera cam = bufferedScene.camera();

        Chunky.getCommonThreads().submit(() -> IntStream.range(0, width).parallel().forEach(i -> {
            Ray ray = new Ray();
            for (int j = 0; j < height; j++) {
                int offset = (j * width + i) * 3;
                cam.calcViewRay(ray, -halfWidth + i * invHeight, -0.5 + j*invHeight);

                rayDirs[offset + 0] = (float) ray.d.x;
                rayDirs[offset + 1] = (float) ray.d.y;
                rayDirs[offset + 2] = (float) ray.d.z;
            }
        })).join();

        return rayDirs;
    }

    private float[] generateJitterLengths(float[] rayDirs) {
        int width = bufferedScene.canvasWidth();
        int height = bufferedScene.canvasHeight();

        double halfWidth = width / (2.0 * height);
        double invHeight = 1.0 / height;

        float[] jitterDirs = new float[width * height * 3];

        Camera cam = bufferedScene.camera();

        Chunky.getCommonThreads().submit(() -> IntStream.range(0, width).parallel().forEach(i -> {
            Ray ray = new Ray();
            for (int j = 0; j < height; j++) {
                int offset = (j * width + i) * 3;
                cam.calcViewRay(ray, -halfWidth + (i+1)*invHeight, -0.5 + (j+1)*invHeight);
                jitterDirs[offset + 0] = (float) ray.d.x - rayDirs[offset + 0];
                jitterDirs[offset + 1] = (float) ray.d.y - rayDirs[offset + 1];
                jitterDirs[offset + 2] = (float) ray.d.z - rayDirs[offset + 2];
            }
        })).join();

        return jitterDirs;
    }

    @Override public void run() {
        try {
            while (!isInterrupted()) {
                ResetReason reason = sceneProvider.awaitSceneStateChange();

                synchronized (bufferedScene) {
                    sceneProvider.withSceneProtected(scene -> {
                        if (reason.overwriteState()) {
                            bufferedScene.copyState(scene);

                            intersectCl.generateSky(bufferedScene);
                        }
                        if (reason == ResetReason.MATERIALS_CHANGED || reason == ResetReason.SCENE_LOADED) {
                            scene.importMaterials();
                            intersectCl.load(bufferedScene, renderTask);
                        }

                        bufferedScene.copyTransients(scene);
                        updateRenderState(scene);

                        if (reason == ResetReason.SCENE_LOADED) {
                            bufferedScene.swapBuffers();

                            sendSceneStatus(bufferedScene.sceneStatus());
                        }
                    });
                }

                if (mode == RenderMode.PREVIEW) {
                    System.out.println("Previewing");
                    previewRender();
                } else {
                    System.out.println("Rendering");

                    int spp, targetSpp;
                    synchronized (bufferedScene) {
                        spp = bufferedScene.spp;
                        targetSpp = bufferedScene.getTargetSpp();
                        if (spp < targetSpp) {
                            updateRenderProgress();
                        }
                    }

                    if (spp < targetSpp) {
                        finalRenderer();
                    } else {
                        sceneProvider.withEditSceneProtected(scene -> {
                            scene.pauseRender();
                            updateRenderState(scene);
                        });
                    }
                }

                if (headless) {
                    break;
                }
            }
        } catch (InterruptedException e) {
            // 3D view was closed.
        } catch (Throwable e) {
            Log.error("Unchecked exception in render manager", e);
        }
    }

    private void previewRender() throws InterruptedException {
        // Start tasktracker
        renderTask.update("Preview", 1, 0, "");

        // Generate camera rays
        float[] rayDirs = generateCameraRays();

        Vector3 origin = bufferedScene.camera().getPosition();
        origin.sub(bufferedScene.getOrigin());

        // Do the rendering
        float[] rendermap = intersectCl.rayTrace(origin, rayDirs, new float[rayDirs.length], random, 1, true, bufferedScene, drawDepth, drawEntities);

        // Copy the samples over
        double[] samples = bufferedScene.getSampleBuffer();
        Chunky.getCommonThreads().submit(() -> Arrays.parallelSetAll(samples, i -> rendermap[i])).join();

        // Tell worker threads to finalize all pixels and exit
        finalizer.finalizeOnce();
        bufferedScene.swapBuffers();

        // Update render status display
        renderListeners.forEach(listener -> {
            listener.setRenderTime(0);
            listener.setSamplesPerSecond(0);
            listener.setSpp(1);
        });

        // Update task tracker
        renderTask.update("Preview", 1, 1, "");

        canvas.repaint();
    }

    private void finalRenderer() throws InterruptedException {
        // Reload everything
        intersectCl.load(bufferedScene, renderTask);
        intersectCl.generateSky(bufferedScene);

        // Start render task
        renderTask.update("Rendering", bufferedScene.getTargetSpp(), bufferedScene.spp);

        // Generate camera rays
        float[] rayDirs = generateCameraRays();
        float[] jitterDirs = generateJitterLengths(rayDirs);

        Vector3 origin = new Vector3(bufferedScene.camera().getPosition());
        origin.sub(bufferedScene.getOrigin());

        double[] samples = bufferedScene.getSampleBuffer();

        // Create ray tracing cache
        GpuRayTracer.RayTraceCache cache = intersectCl.createCache(rayDirs, jitterDirs);

        // Merging task, let it run while the GPU is busy
        ForkJoinTask mergeTask = null;

        // Tell the render workers to continuously finalize all pixels
        if (shouldFinalize) {
            finalizer.finalizeSoon();
        }

        while (bufferedScene.spp < bufferedScene.getTargetSpp()) {
            // Start time
            long frameStart = System.currentTimeMillis();

            // Update stuff
            sceneProvider.withSceneProtected(scene -> {
                synchronized (bufferedScene) {
                    bufferedScene.copyTransients(scene);
                    updateRenderState(scene);
                }
            });

            // Check if render was canceled
            if (mode == RenderMode.PAUSED || sceneProvider.pollSceneStateChange()) {
                finalizer.finalizeOnce();
                bufferedScene.swapBuffers();
                canvas.repaint();
                cache.release();
                return;
            }

            // Do the rendering
            float[] rendermap = intersectCl.rayTrace(origin, random, bufferedScene.getRayDepth(), false, bufferedScene, drawDepth, drawEntities, cache);

            if (mergeTask != null && !mergeTask.isDone()) mergeTask.join();

            // Update the output buffer
            int sppF = bufferedScene.spp;
            mergeTask = Chunky.getCommonThreads().submit(() -> Arrays.parallelSetAll(samples, i ->
                    (samples[i] * sppF + rendermap[i]) / (sppF + 1)));

            // Finalize if necessary
            if (snapshotControl.saveSnapshot(bufferedScene, sppF+1)) {
                mergeTask.join();
                finalizer.finalizeOnce();
            }

            // Update render bar
            synchronized (bufferedScene) {
                bufferedScene.renderTime += System.currentTimeMillis() - frameStart;
                bufferedScene.spp += 1;
                updateRenderProgress();
            }

            // Update the screen
            if (shouldFinalize && !finalizer.isFinalizing()) {
                bufferedScene.swapBuffers();
                canvas.repaint();

                finalizer.finalizeSoon();
            }

            // Update frame complete listener
            frameCompleteListener.accept(bufferedScene, bufferedScene.spp);
        }

        if (mergeTask != null) mergeTask.join();

        // Ensure finalization
        finalizer.finalizeOnce();

        // Update the screen
        bufferedScene.swapBuffers();
        canvas.repaint();

        // Release the GPU memory objects
        cache.release();

        // Inform render is complete
        renderCompleteListener.accept(bufferedScene.renderTime, samplesPerSecond());
    }

    private void updateRenderProgress() {
        double renderTime = bufferedScene.renderTime / 1000.0;

        // Notify progress listener.
        int target = bufferedScene.getTargetSpp();
        long etaSeconds = (long) (((target - bufferedScene.spp) * renderTime) / bufferedScene.spp);
        if (etaSeconds > 0) {
            int seconds = (int) ((etaSeconds) % 60);
            int minutes = (int) ((etaSeconds / 60) % 60);
            int hours = (int) (etaSeconds / 3600);
            String eta = String.format("%d:%02d:%02d", hours, minutes, seconds);
            renderTask.update("Rendering", target, bufferedScene.spp, eta);
        } else {
            renderTask.update("Rendering", target, bufferedScene.spp, "");
        }

        synchronized (this) {
            renderListeners.forEach(listener -> {
                listener.setRenderTime(bufferedScene.renderTime);
                listener.setSamplesPerSecond(samplesPerSecond());
                listener.setSpp(bufferedScene.spp);
            });
        }
    }

    private int samplesPerSecond() {
        int canvasWidth = bufferedScene.canvasWidth();
        int canvasHeight = bufferedScene.canvasHeight();
        long pixelsPerFrame = (long) canvasWidth * canvasHeight;
        double renderTime = bufferedScene.renderTime / 1000.0;
        return (int) ((bufferedScene.spp * pixelsPerFrame) / renderTime);
    }
}
