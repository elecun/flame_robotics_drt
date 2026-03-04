import json

class SimParameterMap:
    def __init__(self, ui, config, console):
        self.ui = ui
        self.config = config
        self.console = console

        self.x_res = 0.01
        self.z_res = 0.01
        self.r_res = 0.1

        self._bind_signals()

    def _bind_signals(self):
        """Bind UI signals for real-time synchronization between sliders and line edits"""
        
        # Sliders
        if hasattr(self.ui, 'slider_positioner_x_pos'):
            self.ui.slider_positioner_x_pos.valueChanged.connect(self._on_slider_x_changed)
        if hasattr(self.ui, 'slider_positioner_z_pos'):
            self.ui.slider_positioner_z_pos.valueChanged.connect(self._on_slider_z_changed)
        if hasattr(self.ui, 'slider_positioner_r_pos'):
            self.ui.slider_positioner_r_pos.valueChanged.connect(self._on_slider_r_changed)

        # LineEdits
        if hasattr(self.ui, 'edit_positioner_x_pos'):
            self.ui.edit_positioner_x_pos.editingFinished.connect(self._on_edit_x_changed)
        if hasattr(self.ui, 'edit_positioner_z_pos'):
            self.ui.edit_positioner_z_pos.editingFinished.connect(self._on_edit_z_changed)
        if hasattr(self.ui, 'edit_positioner_r_pos'):
            self.ui.edit_positioner_r_pos.editingFinished.connect(self._on_edit_r_changed)

    def load_parameters(self, file_path):
        """Load JSON file and configure UI components"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                sim_params = json.load(f)
                self.console.info(f"Loaded simulation parameters from {file_path}")
                self._update_ui_parameters(sim_params)
        except Exception as e:
            self.console.error(f"Failed to load parameters: {e}")

    def _update_ui_parameters(self, params):
        """Update the range and resolution of the sliders based on loaded parameters"""
        self.x_res = params.get('positioner_x_resolution', 0.01)
        self.z_res = params.get('positioner_z_resolution', 0.01)
        self.r_res = params.get('positioner_r_resolution', 0.1)

        x_range = params.get('positioner_x_range', [0.0, 8.0])
        z_range = params.get('positioner_z_range', [0.0, 3.0])
        r_range = params.get('positioner_r_range', [0.0, 360.0])

        if hasattr(self.ui, 'slider_positioner_x_pos'):
            self.ui.slider_positioner_x_pos.setMinimum(0)
            self.ui.slider_positioner_x_pos.setMaximum(int((x_range[1] - x_range[0]) / self.x_res))

        if hasattr(self.ui, 'slider_positioner_z_pos'):
            self.ui.slider_positioner_z_pos.setMinimum(0)
            self.ui.slider_positioner_z_pos.setMaximum(int((z_range[1] - z_range[0]) / self.z_res))

        if hasattr(self.ui, 'slider_positioner_r_pos'):
            self.ui.slider_positioner_r_pos.setMinimum(0)
            self.ui.slider_positioner_r_pos.setMaximum(int((r_range[1] - r_range[0]) / self.r_res))

    # --- Handlers for Sliders -> LineEdits ---

    def _on_slider_x_changed(self, value):
        if hasattr(self.ui, 'edit_positioner_x_pos'):
            real_val = value * self.x_res
            self.ui.edit_positioner_x_pos.setText(f"{real_val:.3f}")

    def _on_slider_z_changed(self, value):
        if hasattr(self.ui, 'edit_positioner_z_pos'):
            real_val = value * self.z_res
            self.ui.edit_positioner_z_pos.setText(f"{real_val:.3f}")

    def _on_slider_r_changed(self, value):
        if hasattr(self.ui, 'edit_positioner_r_pos'):
            real_val = value * self.r_res
            self.ui.edit_positioner_r_pos.setText(f"{real_val:.3f}")

    # --- Handlers for LineEdits -> Sliders ---

    def _on_edit_x_changed(self):
        if hasattr(self.ui, 'edit_positioner_x_pos') and hasattr(self.ui, 'slider_positioner_x_pos'):
            try:
                val = float(self.ui.edit_positioner_x_pos.text())
                self.ui.slider_positioner_x_pos.setValue(int(val / self.x_res))
            except ValueError:
                pass

    def _on_edit_z_changed(self):
        if hasattr(self.ui, 'edit_positioner_z_pos') and hasattr(self.ui, 'slider_positioner_z_pos'):
            try:
                val = float(self.ui.edit_positioner_z_pos.text())
                self.ui.slider_positioner_z_pos.setValue(int(val / self.z_res))
            except ValueError:
                pass

    def _on_edit_r_changed(self):
        if hasattr(self.ui, 'edit_positioner_r_pos') and hasattr(self.ui, 'slider_positioner_r_pos'):
            try:
                val = float(self.ui.edit_positioner_r_pos.text())
                self.ui.slider_positioner_r_pos.setValue(int(val / self.r_res))
            except ValueError:
                pass
