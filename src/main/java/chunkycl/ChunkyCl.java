package chunkycl;

import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Tooltip;
import javafx.scene.layout.VBox;
import se.llbit.chunky.Plugin;
import se.llbit.chunky.main.Chunky;
import se.llbit.chunky.main.ChunkyOptions;
import se.llbit.chunky.renderer.RenderController;
import se.llbit.chunky.ui.ChunkyFx;
import se.llbit.chunky.ui.IntegerAdjuster;
import se.llbit.chunky.ui.render.AdvancedTab;
import se.llbit.chunky.ui.render.RenderControlsTab;
import se.llbit.chunky.ui.render.RenderControlsTabTransformer;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

/**
 * This plugin changes the Chunky path tracing renderer to a gpu based path tracer.
 */
public class ChunkyCl implements Plugin {
    @Override public void attach(Chunky chunky) {
        // Change to GPU renderer
        chunky.setRendererFactory(RenderManagerCl::new);

        RenderControlsTabTransformer prev = chunky.getRenderControlsTabTransformer();
        chunky.setRenderControlsTabTransformer(tabs -> {
            // First, call the previous transformer (this allows other plugins to work).
            List<RenderControlsTab> transformed = new ArrayList<>(prev.apply(tabs));

            for (RenderControlsTab tab: transformed) {
                if (tab instanceof AdvancedTab) {
                    IntegerAdjuster drawDepthAdjuster = new IntegerAdjuster();
                    drawDepthAdjuster.setName("Draw depth");
                    drawDepthAdjuster.setTooltip("Maximum GPU draw distance");
                    drawDepthAdjuster.setRange(1, 1024);
                    drawDepthAdjuster.clampMin();
                    drawDepthAdjuster.set(256);
                    drawDepthAdjuster.onValueChange(value -> {
                        RenderController controller;

                        // Get the render controller through reflection
                        try {
                            Field controllerField = tab.getClass().getDeclaredField("controller");
                            controllerField.setAccessible(true);
                            controller = (RenderController) controllerField.get(tab);
                        } catch (NoSuchFieldException | IllegalAccessException e) {
                            e.printStackTrace();
                            return;
                        }

                        // Set the draw depth
                        ((RenderManagerCl) controller.getRenderer()).setDrawDepth(value);

                        // Force refresh
                        controller.getSceneManager().getScene().refresh();
                    });

                    // Add draw depth adjuster after ray depth
                    ((VBox) ((AdvancedTab) tab).getContent()).getChildren().add(4, drawDepthAdjuster);

                    CheckBox drawEntitiesCheckBox = new CheckBox("Draw Entities");
                    drawEntitiesCheckBox.setTooltip(new Tooltip("Draw entities, disabling may improve performance."));
                    drawEntitiesCheckBox.setSelected(true);
                    drawEntitiesCheckBox.selectedProperty().addListener((observable, oldValue, newValue) -> {
                        RenderController controller;

                        // Get the render controller through reflection
                        try {
                            Field controllerField = tab.getClass().getDeclaredField("controller");
                            controllerField.setAccessible(true);
                            controller = (RenderController) controllerField.get(tab);
                        } catch (NoSuchFieldException | IllegalAccessException e) {
                            e.printStackTrace();
                            return;
                        }

                        // Set drawEntities
                        ((RenderManagerCl) controller.getRenderer()).setDrawEntities(newValue);

                        // Force refresh
                        controller.getSceneManager().getScene().refresh();
                    });

                    // Add drawEntities after draw depth
                    ((VBox) ((AdvancedTab) tab).getContent()).getChildren().add(5, drawEntitiesCheckBox);

                    Button deviceSelectorButton = new Button("Select OpenCL Device");
                    deviceSelectorButton.setOnMouseClicked(event -> {
                        GpuSelector selector = new GpuSelector();
                        selector.show();
                    });

                    // Add OpenCL device selector after CPU Utilization
                    ((VBox) ((AdvancedTab) tab).getContent()).getChildren().add(2, deviceSelectorButton);
                }
            }

            return transformed;
        });

    }

    public static void main(String[] args) throws Exception {
        // Start Chunky normally with this plugin attached.
        Chunky.loadDefaultTextures();
        Chunky chunky = new Chunky(ChunkyOptions.getDefaults());
        new ChunkyCl().attach(chunky);
        ChunkyFx.startChunkyUI(chunky);
    }
}
