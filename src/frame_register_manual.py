"""Interactive 6DoF point-cloud alignment, adapted for Windows."""
import argparse
import json
import os
import platform
from pathlib import Path

# Native Windows has no os.uname().  The overrides are only for WSLg.
if "microsoft" in platform.release().lower() and os.environ.get("DISPLAY"):
    os.environ["WAYLAND_DISPLAY"] = ""
    os.environ["XDG_SESSION_TYPE"] = "x11"
    os.environ.setdefault("GLFW_PLATFORM", "x11")

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from open3d.visualization import gui, rendering
import numpy as np
import open3d as o3d


def euler2mat(yaw, pitch, roll):
    cz, sz = np.cos(yaw), np.sin(yaw)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cx, sx = np.cos(roll), np.sin(roll)
    return np.array(((cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx),
                     (sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx),
                     (-sy, cy*sx, cy*cx)))


def make_tf(tx, ty, tz, yaw, pitch, roll, rotation_center=None):
    """Pose matrix, rotating about rotation_center rather than world origin."""
    rotation = euler2mat(yaw, pitch, roll)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    translation = np.array((tx, ty, tz), dtype=float)
    if rotation_center is not None:
        center = np.asarray(rotation_center, dtype=float)
        translation += center - rotation @ center
    transform[:3, 3] = translation
    return transform


def project_to_profile(points, polyline, thickness):
    """Vectorized profile projection; only invoked on explicit S keypress."""
    xy, z = points[:, :2], points[:, 2]
    result_s, result_z, offset = [], [], 0.0
    for p0, p1 in zip(polyline[:-1], polyline[1:]):
        segment = p1 - p0
        length = np.linalg.norm(segment)
        if length < 1e-8:
            continue
        direction = segment / length
        projected = np.clip((xy - p0) @ direction, 0, length)
        nearest = p0 + projected[:, None] * direction
        mask = np.linalg.norm(xy - nearest, axis=1) < thickness / 2
        result_s.append(offset + projected[mask])
        result_z.append(z[mask])
        offset += length
    # Report the coordinate relative to the profile centre (e.g. -20 to +20 m).
    return (np.concatenate(result_s) - offset / 2, np.concatenate(result_z)) if result_s else (np.empty(0), np.empty(0))


class AlignGUI:
    def __init__(self, args):
        self.profile = np.array(((0, 0), (20, 0), (20, 10)), dtype=float)  # replace with your XY polyline
        self.slice_thickness = args.slice_thickness
        self.translation_step = 0.01  # fixed: 1 cm
        self.rotation_step = np.radians(0.02)  # fixed: 0.02 degrees
        self.output_dir = Path(args.output_dir)
        self.tx = self.ty = self.tz = self.yaw = self.pitch = self.roll = 0.0
        self.ref_raw = self._read_cloud(args.ref)
        self.src_raw = self._read_cloud(args.src)
        # The source-frame bounding-box centre is the vehicle/pivot centre.
        # Profiles follow this centre after every source translation.
        src_points = np.asarray(self.src_raw.points)
        self.src_center = (src_points.min(axis=0) + src_points.max(axis=0)) / 2
        self.profile_half_length = args.profile_half_length
        # Real-time rendering uses reduced clouds. Full-resolution source is
        # retained for accurate final export, avoiding a full transform/copy per key.
        self.ref = self._display_cloud(self.ref_raw, args.display_voxel)
        self.src_base = self._display_cloud(self.src_raw, args.display_voxel)
        self.src_base_points = np.asarray(self.src_base.points).copy()
        self.src = o3d.geometry.PointCloud(self.src_base)
        self.src.paint_uniform_color((0.0, 0.75, 1.0))  # source: high-contrast cyan
        self.app = gui.Application.instance
        self.app.initialize()
        self.window = self.app.create_window("6DoF Align | G Save | R Reset", 1280, 900)
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        ref_material = rendering.MaterialRecord()
        ref_material.shader = "defaultUnlit"
        ref_material.point_size = 1.5
        src_material = rendering.MaterialRecord()
        src_material.shader = "defaultUnlit"
        src_material.point_size = 3.0
        src_material.base_color = (0.0, 0.75, 1.0, 1.0)
        self.scene.scene.add_geometry("reference", self.ref, ref_material)
        self.scene.scene.add_geometry("source", self.src, src_material)
        self.src_material = src_material
        # Include both clouds in the starting view so a separated source frame
        # is still visible and can be moved towards the map.
        all_points = np.vstack((np.asarray(self.ref.points), np.asarray(self.src.points)))
        bounds = o3d.geometry.AxisAlignedBoundingBox(all_points.min(axis=0), all_points.max(axis=0))
        self.scene.setup_camera(60, bounds, bounds.get_center())
        self.figures = [Figure(figsize=(6, 2.4), dpi=110), Figure(figsize=(6, 2.4), dpi=110)]
        self.profile_images = [gui.ImageWidget(), gui.ImageWidget()]
        self.window.add_child(self.scene)
        for image in self.profile_images:
            self.window.add_child(image)
        self.window.set_on_layout(self._layout)
        self.window.set_on_key(self._on_key)
        self._refresh_profiles()
        print(f"Display points: ref {len(self.ref.points):,}/{len(self.ref_raw.points):,}; src {len(self.src.points):,}/{len(self.src_raw.points):,}")

    @staticmethod
    def _read_cloud(path):
        path = Path(path)
        if not path.is_file():
            raise FileNotFoundError(f"Point cloud not found: {path}")
        cloud = o3d.io.read_point_cloud(str(path))
        if cloud.is_empty():
            raise ValueError(f"No points read from: {path}")
        return cloud

    @staticmethod
    def _display_cloud(cloud, voxel):
        return cloud.voxel_down_sample(voxel) if voxel > 0 else o3d.geometry.PointCloud(cloud)

    def _pose_transform(self):
        return make_tf(self.tx, self.ty, self.tz, self.yaw, self.pitch, self.roll, self.src_center)

    def _current_profile_center(self):
        # Rotation occurs around src_center; translation moves that centre.
        return self.src_center[:2] + np.array((self.tx, self.ty))

    def _current_profiles(self):
        cx, cy = self._current_profile_center()
        h = self.profile_half_length
        return {"X axis": np.array(((cx - h, cy), (cx + h, cy))),
                "Y axis": np.array(((cx, cy - h), (cx, cy + h)))}

    def update_src(self):
        transform = self._pose_transform()
        base_h = np.column_stack((self.src_base_points, np.ones(len(self.src_base_points))))
        self.src.points = o3d.utility.Vector3dVector((base_h @ transform.T)[:, :3])
        self.src.paint_uniform_color((0.0, 0.75, 1.0))
        self.scene.scene.remove_geometry("source")
        self.scene.scene.add_geometry("source", self.src, self.src_material)
        self._refresh_profiles()
    def _change(self, field, delta):
        setattr(self, field, getattr(self, field) + delta)
        self.update_src()
        return False

    def _layout(self, layout_context):
        rect = self.window.content_rect
        scene_height = int(rect.height * 0.62)
        self.scene.frame = gui.Rect(rect.x, rect.y, rect.width, scene_height)
        panel_y, panel_h = rect.y + scene_height, rect.height - scene_height
        half_w = rect.width // 2
        self.profile_images[0].frame = gui.Rect(rect.x, panel_y, half_w, panel_h)
        self.profile_images[1].frame = gui.Rect(rect.x + half_w, panel_y, rect.width - half_w, panel_h)

    def _profile_colors(self, cloud, fallback):
        """Return cloud RGB for QGIS-like profile rendering, or a fallback."""
        colors = np.asarray(cloud.colors)
        if len(colors) == len(cloud.points):
            return np.clip(colors, 0.0, 1.0)
        return np.broadcast_to(fallback, (len(cloud.points), 3))

    def _refresh_profiles(self):
        transform = self._pose_transform()
        base_h = np.column_stack((self.src_base_points, np.ones(len(self.src_base_points))))
        src_points = (base_h @ transform.T)[:, :3]
        ref_points = np.asarray(self.ref.points)
        ref_colors = self._profile_colors(self.ref, (0.68, 0.72, 0.76))
        src_colors = np.broadcast_to((0.0, 0.84, 1.0), (len(src_points), 3))
        for figure, image, (name, profile) in zip(self.figures, self.profile_images, self._current_profiles().items()):
            figure.clear()
            figure.patch.set_facecolor("#16191d")
            ax = figure.add_subplot(111)
            ax.set_facecolor("#16191d")
            s_ref, z_ref, c_ref = self._project_colored(ref_points, ref_colors, profile)
            s_src, z_src, c_src = self._project_colored(src_points, src_colors, profile)
            # Pixel-like, dense rendering on a dark canvas matches QGIS profile
            # inspection much better than the former large red/blue markers.
            ax.scatter(s_ref, z_ref, s=1.2, c=c_ref, marker=".", linewidths=0, alpha=0.92, rasterized=True)
            ax.scatter(s_src, z_src, s=2.2, c=c_src, marker=".", linewidths=0, alpha=0.96, rasterized=True)
            ax.axvline(0, color="#ffca28", linewidth=0.7, alpha=0.8)
            ax.set(title=f"{name} profile | centre ({self._current_profile_center()[0]:.2f}, {self._current_profile_center()[1]:.2f})",
                   xlabel="Distance from source centre (m)", ylabel="Elevation Z (m)", xlim=(-20, 20))
            ax.tick_params(colors="#d8dde3", labelsize=8)
            ax.xaxis.label.set_color("#d8dde3"); ax.yaxis.label.set_color("#d8dde3"); ax.title.set_color("#f0f3f5")
            for spine in ax.spines.values():
                spine.set_color("#59636e")
            ax.grid(color="#3f4852", linewidth=0.45, alpha=0.65)
            figure.tight_layout(pad=0.7)
            canvas = FigureCanvasAgg(figure); canvas.draw()
            rgba = np.asarray(canvas.buffer_rgba())
            image.update_image(o3d.geometry.Image(np.ascontiguousarray(rgba[:, :, :3])))

    def _project_colored(self, points, colors, polyline):
        """Profile-project points and carry their corresponding RGB values."""
        xy, z = points[:, :2], points[:, 2]
        all_s, all_z, all_colors, offset = [], [], [], 0.0
        for p0, p1 in zip(polyline[:-1], polyline[1:]):
            segment = p1 - p0
            length = np.linalg.norm(segment)
            if length < 1e-8:
                continue
            direction = segment / length
            projected = np.clip((xy - p0) @ direction, 0, length)
            nearest = p0 + projected[:, None] * direction
            mask = np.linalg.norm(xy - nearest, axis=1) < self.slice_thickness / 2
            all_s.append(offset + projected[mask]); all_z.append(z[mask]); all_colors.append(colors[mask])
            offset += length
        if not all_s:
            return np.empty(0), np.empty(0), np.empty((0, 3))
        return np.concatenate(all_s) - offset / 2, np.concatenate(all_z), np.concatenate(all_colors)
    def _on_key(self, event):
        # Window.set_on_key requires a bool in Open3D 0.19 (not EventCallbackResult).
        if event.type != gui.KeyEvent.DOWN:
            return False
        key = event.key
        changes = {gui.KeyName.A: ("tx", -self.translation_step), gui.KeyName.D: ("tx", self.translation_step),
                   gui.KeyName.W: ("ty", self.translation_step), gui.KeyName.S: ("ty", -self.translation_step),
                   gui.KeyName.Q: ("tz", self.translation_step), gui.KeyName.E: ("tz", -self.translation_step),
                   gui.KeyName.ONE: ("yaw", self.rotation_step), gui.KeyName.THREE: ("yaw", -self.rotation_step),
                   gui.KeyName.FOUR: ("pitch", self.rotation_step), gui.KeyName.SIX: ("pitch", -self.rotation_step),
                   gui.KeyName.SEVEN: ("roll", self.rotation_step), gui.KeyName.NINE: ("roll", -self.rotation_step)}
        if key in changes:
            self._change(*changes[key])
            return True
        if key == gui.KeyName.G:
            self.confirm_save()
            return True
        if key == gui.KeyName.R:
            self.reset()
            return True
        return False

    def confirm_save(self):
        dialog = gui.Dialog("Confirm save")
        em = int(self.window.theme.font_size)
        content = gui.Vert(em, gui.Margins(em, em, em, em))
        content.add_child(gui.Label(f"Save human_gt.json and aligned_frame.pcd to:\n{self.output_dir}?"))
        buttons = gui.Horiz(em)
        cancel = gui.Button("Cancel")
        confirm = gui.Button("Save")
        cancel.set_on_clicked(lambda: self.window.close_dialog())
        def do_save():
            self.window.close_dialog()
            self.save_gt()
        confirm.set_on_clicked(do_save)
        buttons.add_child(cancel)
        buttons.add_child(confirm)
        content.add_child(buttons)
        dialog.add_child(content)
        self.window.show_dialog(dialog)
    def save_gt(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        transform = self._pose_transform()
        ypr = np.degrees((self.yaw, self.pitch, self.roll))
        data = {"transform_matrix": transform.tolist(), "tx": self.tx, "ty": self.ty, "tz": self.tz,
                "yaw_deg": ypr[0], "pitch_deg": ypr[1], "roll_deg": ypr[2]}
        (self.output_dir / "human_gt.json").write_text(json.dumps(data, indent=2), encoding="utf-8")
        aligned = o3d.geometry.PointCloud(self.src_raw)
        aligned.transform(transform)
        o3d.io.write_point_cloud(str(self.output_dir / "aligned_frame.pcd"), aligned)
        print(f"[Saved] {self.output_dir}")

    def reset(self):
        self.tx = self.ty = self.tz = self.yaw = self.pitch = self.roll = 0.0
        self.update_src()
        return False

    def run(self):
        print("A/D: X | W/S: Y | Q/E: Z, step 1 cm | G: confirm save")
        print("1/3: yaw | 4/6: pitch | 7/9: roll, step 0.02 deg | G: save | R: reset")
        self.app.run()

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref", required=True, help="Reference/map PCD path")
    parser.add_argument("--src", required=True, help="Source/frame PCD path")
    parser.add_argument("--output-dir", default=".")
    parser.add_argument("--display-voxel", type=float, default=0.10, help="Display voxel size in metres; 0 retains every point")
    parser.add_argument("--slice-thickness", type=float, default=0.1)
    parser.add_argument("--profile-half-length", type=float, default=20.0, help="Profile range on each side of source centre (m)")
    parser.add_argument("--translation-step", type=float, default=0.01)
    parser.add_argument("--rotation-step-deg", type=float, default=0.1)
    return parser.parse_args()


if __name__ == "__main__":
    AlignGUI(parse_args()).run()












