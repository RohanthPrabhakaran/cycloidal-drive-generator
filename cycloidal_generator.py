"""
Cycloidal Drive Generator with GUI
Refactored: 2025-09-17
Interactive GUI for cycloidal drive design. Adjust parameters with sliders and refresh the drawing and DXF.
"""

import numpy as np
import math
import ezdxf
from shapely.geometry import LinearRing
import matplotlib.pyplot as plt
import cv2
import tkinter as tk
from tkinter import ttk
import time

# --- Core cycloidal drive logic as a function ---
def generate_cycloidal(overall_diameter, number_of_pins, ecc_factor=0.6, auto_pin_size=True, adv_params=None):
    # Use advanced parameters if provided
    min_pin_fraction = 0.10
    max_pin_fraction = 0.22  # never exceed this fraction of overall diameter
    if adv_params:
        pin_diameter = adv_params['pin_diameter'] if adv_params['pin_diameter'] > 0 else min_pin_fraction * overall_diameter
        pin_circle_radius = adv_params['pin_circle_radius'] if adv_params['pin_circle_radius'] > 0 else (overall_diameter - pin_diameter) / 2.0
        center_hole_percent = adv_params['center_hole_percent']
        output_hole_percent = adv_params['output_hole_percent']
        output_bolt_circle_percent = adv_params['output_bolt_circle_percent']
    else:
        if auto_pin_size:
            # For low pin counts, increase pin size up to max_pin_fraction
            pin_fraction = min_pin_fraction + (max_pin_fraction - min_pin_fraction) * max(0, (10-number_of_pins)/10)
            pin_diameter = pin_fraction * overall_diameter
            pin_diameter = min(pin_diameter, max_pin_fraction * overall_diameter)
        else:
            pin_diameter = min_pin_fraction * overall_diameter
        pin_circle_radius = (overall_diameter - pin_diameter) / 2.0
        center_hole_percent = 0.30
        output_hole_percent = 0.10
        output_bolt_circle_percent = 0.6
    inner_pins = number_of_pins - 1
    samples = 4000
    output_holes_count = inner_pins
    output_pin_angle_offset_deg = (360/(output_holes_count))/2

    def rotor_param(R, E, Rr, N, t):
        m = 1 - N
        denom = (R / (E * N)) - np.cos(m * t)
        safe_denom = np.where(np.abs(denom) < 1e-9, np.sign(denom) * 1e-9 + denom, denom)
        ang = np.arctan2(np.sin(m * t), safe_denom)
        x = R * np.cos(t) - Rr * np.cos(t + ang) - E * np.cos(N * t)
        y = -R * np.sin(t) + Rr * np.sin(t + ang) + E * np.sin(N * t)
        return x, y

    rolling_circle_radius = pin_circle_radius / number_of_pins
    R = (number_of_pins - 1) * rolling_circle_radius
    E = ecc_factor * rolling_circle_radius
    Rr = pin_diameter / 2.0
    t_ext = np.linspace(np.deg2rad(-30.0), np.deg2rad(210.0), samples)
    x_ext, y_ext = rotor_param(R, E, Rr, number_of_pins, t_ext)
    crossings = np.where(np.diff(np.sign(y_ext)) != 0)[0]
    if len(crossings) >= 2:
        start_idx = crossings[0]
        end_idx = crossings[-1]
    else:
        start_idx = 0
        end_idx = len(t_ext) - 1
    half_x = x_ext[start_idx:end_idx + 1]
    half_y = y_ext[start_idx:end_idx + 1]
    half_pts = np.column_stack((half_x, half_y))
    mirrored_pts = np.column_stack((half_x[::-1], -half_y[::-1]))
    full_pts = np.vstack((half_pts, mirrored_pts[1:-1]))
    ring = LinearRing(full_pts)
    if not ring.is_ring:
        full_pts = np.vstack((full_pts, full_pts[0]))
    radii = np.hypot(full_pts[:,0], full_pts[:,1])
    max_radius = float(np.max(radii))
    rotor_diameter = 2.0 * max_radius
    center_hole_d = center_hole_percent * rotor_diameter
    output_hole_d = output_hole_percent * rotor_diameter
    output_bolt_radius = output_bolt_circle_percent * max_radius
    rotor_offset_x = E
    rotor_offset_y = 0
    # DXF generation
    doc = ezdxf.new(dxfversion='R2010')
    # Set DXF units to millimeters (1 = inches, 4 = millimeters)
    doc.header["$INSUNITS"] = 4
    msp = doc.modelspace()
    # Draw outer circle (gearbox center)
    msp.add_circle((0, 0), overall_diameter/2.0, dxfattribs={'color': 7})
    # Draw gearbox center circle (main shaft bearing at (0,0))
    msp.add_circle((0, 0), 0.15*overall_diameter/2.0, dxfattribs={'color': 5})
    msp.add_point((0, 0), dxfattribs={'color': 3})
    # Draw pin circles and mark centers
    for ang_deg in np.linspace(0, 360, num=number_of_pins, endpoint=False):
        a = math.radians(ang_deg)
        px = pin_circle_radius * math.cos(a)
        py = pin_circle_radius * math.sin(a)
        msp.add_circle((px, py), pin_diameter/2.0, dxfattribs={'color': 4})
        msp.add_point((px, py), dxfattribs={'color': 2})
    # Draw rotor profile (as polyline), offset by E
    points = [(float(x+rotor_offset_x), float(y+rotor_offset_y)) for x, y in full_pts]
    msp.add_lwpolyline(points, close=True, dxfattribs={'color': 1})
    # Draw rotor center hole (at offset)
    msp.add_circle((rotor_offset_x, rotor_offset_y), center_hole_d/2.0, dxfattribs={'color': 5})
    msp.add_point((rotor_offset_x, rotor_offset_y), dxfattribs={'color': 6})
    # Draw output holes (at offset)
    for i in range(output_holes_count):
        ang = math.radians(output_pin_angle_offset_deg + (360 / output_holes_count) * i)
        ox = output_bolt_radius * math.cos(ang) + rotor_offset_x
        oy = output_bolt_radius * math.sin(ang) + rotor_offset_y
        msp.add_circle((ox, oy), output_hole_d/2.0, dxfattribs={'color': 6})
        msp.add_point((ox, oy), dxfattribs={'color': 3})
    filename = f"cycloidal_drive_{overall_diameter:.1f}mm_{number_of_pins}pins.dxf"
    doc.saveas(filename)
    # PNG preview
    fig, ax = plt.subplots(figsize=(8,8))
    ax.set_aspect('equal', 'box')
    ax.axis('off')
    # Draw outer circle (gearbox center)
    outer = plt.Circle((0,0), overall_diameter/2.0, fill=False, color='black', linestyle='--', linewidth=1.2)
    ax.add_patch(outer)
    # Draw gearbox center circle
    ax.add_patch(plt.Circle((0,0), 0.15*overall_diameter/2.0, fill=False, color='blue', linewidth=1.0))
    ax.plot(0, 0, 'kx', markersize=8)
    # Draw pin circles
    for ang_deg in np.linspace(0, 360, num=number_of_pins, endpoint=False):
        a = math.radians(ang_deg)
        px = pin_circle_radius * math.cos(a)
        py = pin_circle_radius * math.sin(a)
        ax.add_patch(plt.Circle((px, py), pin_diameter/2.0, fill=False, color='gray', linewidth=0.8))
        ax.plot(px, py, 'go', markersize=4)
    # Draw rotor profile (offset)
    ax.plot(full_pts[:,0]+rotor_offset_x, full_pts[:,1]+rotor_offset_y, '-r', linewidth=1.5)
    # Draw rotor center hole (offset)
    ax.add_patch(plt.Circle((rotor_offset_x, rotor_offset_y), center_hole_d/2.0, fill=False, color='green', linewidth=1.0))
    ax.plot(rotor_offset_x, rotor_offset_y, 'rx', markersize=8)
    # Draw output holes (offset)
    for i in range(output_holes_count):
        ang = math.radians(output_pin_angle_offset_deg + (360 / output_holes_count) * i)
        ox = output_bolt_radius * math.cos(ang) + rotor_offset_x
        oy = output_bolt_radius * math.sin(ang) + rotor_offset_y
        ax.add_patch(plt.Circle((ox, oy), output_hole_d/2.0, fill=False, color='purple', linewidth=1.0))
        ax.plot(ox, oy, 'mo', markersize=4)
    plt.savefig('cycloidal_preview.png', bbox_inches='tight', pad_inches=0.1, dpi=300)
    plt.close()
    return filename, 'cycloidal_preview.png'

# --- Tkinter GUI ---
class CycloidalGUI:
    def __init__(self, root):
        self.root = root
        root.title("Cycloidal Drive Generator")
        self.frame = ttk.Frame(root, padding=10)
        self.frame.grid(row=0, column=0, sticky="nsew")
        root.geometry("750x250")
        # Inputs and buttons
        self.diam_var = tk.DoubleVar(value=150.0)
        self.pins_var = tk.IntVar(value=7)
        self.ecc_var = tk.DoubleVar(value=0.6)
        self.auto_pin_var = tk.BooleanVar(value=True)
        # Diameter
        ttk.Label(self.frame, text="Overall Diameter (mm)").grid(row=0, column=0, sticky="w")
        self.diam_entry = ttk.Entry(self.frame, textvariable=self.diam_var, width=8)
        self.diam_entry.grid(row=0, column=1)
        ttk.Button(self.frame, text="-", width=2, command=lambda: self.adjust(self.diam_var, -5, 50, 300)).grid(row=0, column=2)
        ttk.Button(self.frame, text="+", width=2, command=lambda: self.adjust(self.diam_var, 5, 50, 300)).grid(row=0, column=3)
        # Pins
        ttk.Label(self.frame, text="Number of Pins").grid(row=1, column=0, sticky="w")
        self.pins_entry = ttk.Entry(self.frame, textvariable=self.pins_var, width=8)
        self.pins_entry.grid(row=1, column=1)
        ttk.Button(self.frame, text="-", width=2, command=lambda: self.adjust(self.pins_var, -1, 5, 20)).grid(row=1, column=2)
        ttk.Button(self.frame, text="+", width=2, command=lambda: self.adjust(self.pins_var, 1, 5, 20)).grid(row=1, column=3)
        # Eccentricity
        ttk.Label(self.frame, text="Eccentricity Factor").grid(row=2, column=0, sticky="w")
        self.ecc_entry = ttk.Entry(self.frame, textvariable=self.ecc_var, width=8)
        self.ecc_entry.grid(row=2, column=1)
        ttk.Button(self.frame, text="-", width=2, command=lambda: self.adjust(self.ecc_var, -0.01, 0.3, 0.8)).grid(row=2, column=2)
        ttk.Button(self.frame, text="+", width=2, command=lambda: self.adjust(self.ecc_var, 0.01, 0.3, 0.8)).grid(row=2, column=3)
        # Auto Pin Size
        self.auto_pin_check = ttk.Checkbutton(self.frame, text="Auto Pin Size", variable=self.auto_pin_var)
        self.auto_pin_check.grid(row=5, column=0, columnspan=2, sticky="w")

        # Advanced Options toggle
        self.advanced_var = tk.BooleanVar(value=False)
        self.advanced_check = ttk.Checkbutton(self.frame, text="Advanced Options", variable=self.advanced_var, command=self.toggle_advanced)
        self.advanced_check.grid(row=5, column=2, columnspan=2, sticky="w")


        # Advanced parameter variables
        self.pin_diam_var = tk.DoubleVar(value=0.0)
        self.pin_circle_radius_var = tk.DoubleVar(value=0.0)
        self.center_hole_percent_var = tk.DoubleVar(value=0.30)
        self.output_hole_percent_var = tk.DoubleVar(value=0.10)
        self.output_bolt_circle_percent_var = tk.DoubleVar(value=0.6)

        # Advanced parameter widgets (hidden by default)
        self.advanced_widgets = []
        row_advanced = 6
        self.advanced_widgets.append(ttk.Label(self.frame, text="Pin Diameter (mm)"))
        self.advanced_widgets[-1].grid(row=row_advanced, column=0, sticky="w")
        self.advanced_widgets.append(ttk.Entry(self.frame, textvariable=self.pin_diam_var, width=8))
        self.advanced_widgets[-1].grid(row=row_advanced, column=1)
        self.advanced_widgets.append(ttk.Label(self.frame, text="Pin Circle Radius (mm)"))
        self.advanced_widgets[-1].grid(row=row_advanced, column=2, sticky="w")
        self.advanced_widgets.append(ttk.Entry(self.frame, textvariable=self.pin_circle_radius_var, width=8))
        self.advanced_widgets[-1].grid(row=row_advanced, column=3)
        row_advanced += 1
        self.advanced_widgets.append(ttk.Label(self.frame, text="Center Hole %"))
        self.advanced_widgets[-1].grid(row=row_advanced, column=0, sticky="w")
        self.advanced_widgets.append(ttk.Entry(self.frame, textvariable=self.center_hole_percent_var, width=8))
        self.advanced_widgets[-1].grid(row=row_advanced, column=1)
        self.advanced_widgets.append(ttk.Label(self.frame, text="Output Hole %"))
        self.advanced_widgets[-1].grid(row=row_advanced, column=2, sticky="w")
        self.advanced_widgets.append(ttk.Entry(self.frame, textvariable=self.output_hole_percent_var, width=8))
        self.advanced_widgets[-1].grid(row=row_advanced, column=3)
        row_advanced += 1
        self.advanced_widgets.append(ttk.Label(self.frame, text="Output Bolt Circle %"))
        self.advanced_widgets[-1].grid(row=row_advanced, column=0, sticky="w")
        self.advanced_widgets.append(ttk.Entry(self.frame, textvariable=self.output_bolt_circle_percent_var, width=8))
        self.advanced_widgets[-1].grid(row=row_advanced, column=1)

        # Hide advanced widgets by default
        for widget in self.advanced_widgets:
            widget.grid_remove()

        # Button
        self.refresh_btn = ttk.Button(self.frame, text="Generate & Preview", command=self.refresh)
        self.refresh_btn.grid(row=3, column=0, columnspan=4, pady=10)
        # Status and time
        self.status = ttk.Label(self.frame, text="Ready.")
        self.status.grid(row=4, column=0, columnspan=2)
        self.time_label = ttk.Label(self.frame, text="Time: --:--:--")
        self.time_label.grid(row=4, column=2, columnspan=2, sticky="e")
        self.frame.columnconfigure(1, weight=1)
        self.update_labels()

    def toggle_advanced(self):
        # When enabling advanced, populate fields with current auto values
        if self.advanced_var.get():
            # Calculate auto values
            d = self.diam_var.get()
            n = int(self.pins_var.get())
            min_pin_fraction = 0.10
            max_pin_fraction = 0.22
            if self.auto_pin_var.get():
                pin_fraction = min_pin_fraction + (max_pin_fraction - min_pin_fraction) * max(0, (10-n)/10)
                pin_diameter = pin_fraction * d
                pin_diameter = min(pin_diameter, max_pin_fraction * d)
            else:
                pin_diameter = min_pin_fraction * d
            pin_circle_radius = (d - pin_diameter) / 2.0
            center_hole_percent = 0.30
            output_hole_percent = 0.10
            output_bolt_circle_percent = 0.6
            self.pin_diam_var.set(round(pin_diameter, 4))
            self.pin_circle_radius_var.set(round(pin_circle_radius, 4))
            self.center_hole_percent_var.set(center_hole_percent)
            self.output_hole_percent_var.set(output_hole_percent)
            self.output_bolt_circle_percent_var.set(output_bolt_circle_percent)
        # Show/hide advanced widgets
        for widget in self.advanced_widgets:
            if self.advanced_var.get():
                widget.grid()
            else:
                widget.grid_remove()
    def adjust(self, var, delta, vmin, vmax):
        val = var.get() + delta
        if isinstance(var, tk.IntVar):
            val = int(round(val))
        val = max(vmin, min(val, vmax))
        var.set(val)
        self.update_labels()
    def update_labels(self):
        self.time_label.config(text=f"Time: {time.strftime('%H:%M:%S')}")
    def refresh(self):
        d = self.diam_var.get()
        n = int(self.pins_var.get())
        e = self.ecc_var.get()
        auto_pin = self.auto_pin_var.get()
        advanced = self.advanced_var.get()
        adv_params = None
        if advanced:
            adv_params = {
                'pin_diameter': self.pin_diam_var.get(),
                'pin_circle_radius': self.pin_circle_radius_var.get(),
                'center_hole_percent': self.center_hole_percent_var.get(),
                'output_hole_percent': self.output_hole_percent_var.get(),
                'output_bolt_circle_percent': self.output_bolt_circle_percent_var.get(),
            }
        self.status.config(text="Generating...")
        self.root.update_idletasks()
        dxf_file, png_file = generate_cycloidal(d, n, e, auto_pin, adv_params)
        try:
            cv2.destroyAllWindows()
            img = cv2.imread(png_file)
            max_dim = 600
            h, w = img.shape[:2]
            scale = min(max_dim / h, max_dim / w, 1.0)
            if scale < 1.0:
                img = cv2.resize(img, (int(w*scale), int(h*scale)), interpolation=cv2.INTER_AREA)
            cv2.imshow('Cycloidal Drive Preview', img)
            # Block until window is closed
            while True:
                if cv2.getWindowProperty('Cycloidal Drive Preview', cv2.WND_PROP_VISIBLE) < 1:
                    break
                if cv2.waitKey(100) == 27:
                    break
            cv2.destroyAllWindows()
        except Exception as ex:
            self.status.config(text=f"Preview error: {ex}")
        self.status.config(text=f"DXF saved as {dxf_file}")
        self.update_labels()

if __name__ == "__main__":
    root = tk.Tk()
    app = CycloidalGUI(root)
    root.mainloop()
    cv2.destroyAllWindows()
