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
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.stream.IntStream;

public class RenderManagerCl extends Thread implements Renderer {
    private static final Repaintable EMPTY_CANVAS = () -> {};
    private Repaintable canvas = EMPTY_CANVAS;

    private Finalizer finalizer;
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

    public static final GpuRayTracer intersectCl = new GpuRayTracer();

    public RenderManagerCl(RenderContext context, boolean headless) {
        super("Render Manager");

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

                        if (reason == ResetReason.SCENE_LOADED) {
                            bufferedScene.swapBuffers();

                            String sceneStatus = bufferedScene.sceneStatus();
                            synchronized (sceneListeners) {
                                for (SceneStatusListener listener : sceneListeners) {
                                    listener.sceneStatus(sceneStatus);
                                }
                            }
                        }
                    });

                    mode = bufferedScene.getMode();
                }

                if (mode == RenderMode.PREVIEW) {

                    System.out.println("Previewing");

                    previewRender();

                    renderTask.update("Preview", 1, 1, "");
                } else {
                    System.out.println("Rendering");

                    int targetSpp;
                    targetSpp = bufferedScene.getTargetSpp();

                    intersectCl.load(bufferedScene, renderTask);

                    finalRenderer(targetSpp, renderTask);
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
        // Generate camera starting rays
        int width = bufferedScene.canvasWidth();
        int height = bufferedScene.canvasHeight();

        double halfWidth = width / (2.0 * height);
        double invHeight = 1.0 / height;

        float[] rayPos = new float[width * height * 3];
        float[] rayDirs = new float[rayPos.length];

        Vector3i sceneOrigin = bufferedScene.getOrigin();

        Camera cam = bufferedScene.camera();

        Chunky.getCommonThreads().submit(() -> IntStream.range(0, width).parallel().forEach(i -> {
            Ray ray = new Ray();
            for (int j = 0; j < height; j++) {
                int offset = (j * width + i) * 3;
                cam.calcViewRay(ray, -halfWidth + i*invHeight, -0.5 + j*invHeight);

                rayPos[offset + 0] = (float) (ray.o.x - sceneOrigin.x);
                rayPos[offset + 1] = (float) (ray.o.y - sceneOrigin.y);
                rayPos[offset + 2] = (float) (ray.o.z - sceneOrigin.z);

                rayDirs[offset + 0] = (float) ray.d.x;
                rayDirs[offset + 1] = (float) ray.d.y;
                rayDirs[offset + 2] = (float) ray.d.z;
            }
        })).join();

        double[] samples = bufferedScene.getSampleBuffer();

        // Do the rendering
        float[] depthmap = intersectCl.rayTrace(rayPos, rayDirs, new float[rayDirs.length], random, 1, true, bufferedScene, drawDepth, drawEntities);

        for (int i = 0; i < depthmap.length; i++) {
            samples[i] = depthmap[i];
        }

        // Tell worker threads to finalize all pixels and exit
        finalizer.finalizeOnce();
        bufferedScene.swapBuffers();
        canvas.repaint();
    }

    private void finalRenderer(int targetSpp, TaskTracker.Task renderTask) throws InterruptedException {
        renderTask.update("Rendering", targetSpp, 0);

        // Generate camera rays
        int width = bufferedScene.canvasWidth();
        int height = bufferedScene.canvasHeight();

        double halfWidth = width / (2.0 * height);
        double invHeight = 1.0 / height;

        float[] rayPos = new float[width * height * 3];
        float[] rayDirs = new float[rayPos.length];
        float[] jitterDirs = new float[rayDirs.length];

        Camera cam = bufferedScene.camera();

        // Render sky
        intersectCl.generateSky(bufferedScene);

        Vector3i sceneOrigin = bufferedScene.getOrigin();

        double[] samples = bufferedScene.getSampleBuffer();

        long startTime = System.currentTimeMillis();
        long updateTime = startTime;

        // Tell the render workers to continuously finalize all pixels
        finalizer.finalizeSoon();

        // Generate ray directions
        Chunky.getCommonThreads().submit(() -> IntStream.range(0, width).parallel().forEach(i -> {
            Ray ray = new Ray();
            for (int j = 0; j < height; j++) {
                int offset = (j * width + i) * 3;
                cam.calcViewRay(ray, -halfWidth + i*invHeight, -0.5 + j*invHeight);

                rayPos[offset + 0] = (float) (ray.o.x - sceneOrigin.x);
                rayPos[offset + 1] = (float) (ray.o.y - sceneOrigin.y);
                rayPos[offset + 2] = (float) (ray.o.z - sceneOrigin.z);

                rayDirs[offset + 0] = (float) ray.d.x;
                rayDirs[offset + 1] = (float) ray.d.y;
                rayDirs[offset + 2] = (float) ray.d.z;
            }
        })).join();

        // Generate jitter lengths
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

        for (int sample = bufferedScene.spp; sample < targetSpp; sample++) {

            // Do the rendering
            float[] rendermap = intersectCl.rayTrace(rayPos, rayDirs, jitterDirs, random, bufferedScene.getRayDepth(), false, bufferedScene, drawDepth, drawEntities);

            // Update the output buffer
            for (int i = 0; i < rendermap.length; i++) {
                samples[i] = (samples[i] * bufferedScene.spp + rendermap[i]) / (bufferedScene.spp + 1);
            }

            // Update render bar
            bufferedScene.renderTime = System.currentTimeMillis() - startTime;
            bufferedScene.spp = sample + 1;
            updateRenderProgress();

            // Update the screen
            if (!finalizer.isFinalizing()) {
                bufferedScene.swapBuffers();
                canvas.repaint();

                finalizer.finalizeSoon();
            }

            // Update frame complete listener
            if (sample % 32 == 0 || System.currentTimeMillis() > updateTime + 1000) {
                frameCompleteListener.accept(bufferedScene, sample);
                updateTime = System.currentTimeMillis();
            }

            // Check if render was canceled
            if (mode == RenderMode.PAUSED || sceneProvider.pollSceneStateChange()) {
                break;
            }
        }

        // Wait for render workers to finish
        finalizer.stopNow();

        // Ensure finalization
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                bufferedScene.finalizePixel(i, j);
            }
        }

        // Update the screen
        bufferedScene.swapBuffers();
        canvas.repaint();

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
