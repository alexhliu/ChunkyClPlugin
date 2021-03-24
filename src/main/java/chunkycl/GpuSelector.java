package chunkycl;

import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.collections.FXCollections;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import org.jocl.CL;
import org.jocl.cl_device_id;
import se.llbit.chunky.PersistentSettings;
import se.llbit.log.Log;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Arrays;

public class GpuSelector extends Stage {
    private Scene scene;

    @SuppressWarnings("unchecked")
    public GpuSelector() {
        // Build scene
        GpuRayTracer tracer = GpuRayTracer.getTracer();
        ClDevice[] devices = new ClDevice[tracer.devices.length];
        for (int i = 0; i < devices.length; i++) {
            devices[i] = new ClDevice(tracer.devices[i], i);
        }

        TableView<ClDevice> table = new TableView<>();
        table.setPrefWidth(433);
        table.setPrefHeight(200);
        table.setItems(FXCollections.observableList(Arrays.asList(devices)));

        TableColumn<ClDevice, String> nameCol = new TableColumn<>("Device Name");
        nameCol.setCellValueFactory(dev -> new SimpleStringProperty(dev.getValue().name));

        TableColumn<ClDevice, Double> computeCol = new TableColumn<>("Compute Capacity");
        computeCol.setCellValueFactory(dev -> new SimpleDoubleProperty(dev.getValue().computeCapacity).asObject());

        table.getColumns().setAll(nameCol, computeCol);

        VBox box = new VBox();
        VBox.setVgrow(table, Priority.ALWAYS);
        box.setSpacing(10);
        box.setPadding(new Insets(10));

        box.getChildren().add(new Label("Select OpenCL device to use:"));
        box.getChildren().add(table);

        HBox buttons = new HBox();
        buttons.setAlignment(Pos.TOP_RIGHT);
        buttons.setSpacing(10);

        Button cancelButton = new Button("Cancel");
        cancelButton.setOnMouseClicked(event -> this.close());
        buttons.getChildren().add(cancelButton);

        Button selectButton = new Button("Select Device");
        selectButton.setDefaultButton(true);
        selectButton.setTooltip(new Tooltip("Restart Chunky for changes to take effect."));
        selectButton.setOnMouseClicked(event -> {
            if (!table.getSelectionModel().isEmpty()) {
                PersistentSettings.settings.setInt("clDevice", table.getSelectionModel().getSelectedItem().index);

                // Save with reflection
                try {
                    Method method = PersistentSettings.class.getDeclaredMethod("save");
                    method.setAccessible(true);
                    method.invoke(null);
                } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException e) {
                    Log.warn("Error saving changes", e);
                }

                this.close();
            }
        });
        buttons.getChildren().add(selectButton);

        box.getChildren().add(buttons);

        scene = new Scene(box);

        // Apply scene
        this.setTitle("Select OpenCL Device");
        this.setScene(scene);
    }

    private static class ClDevice {
        protected final String name;
        protected final double computeCapacity;
        protected final int index;

        public ClDevice(cl_device_id device, int index) {
            long computeSpeed = (long) GpuRayTracer.getInts(device, CL.CL_DEVICE_MAX_CLOCK_FREQUENCY, 1)[0] *
                                (long) GpuRayTracer.getInts(device, CL.CL_DEVICE_MAX_COMPUTE_UNITS, 1)[0] * 32;

            name = GpuRayTracer.getString(device, CL.CL_DEVICE_NAME);
            computeCapacity = computeSpeed/1000.0;  // Approximate GFlops
            this.index = index;
        }
    }
}
