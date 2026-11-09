# MAS248 Lecture 9 — Student starter v0.4.0

**Student assignment:** Implement your own `scripts/run_picture.py` and register its
console entry point in `setup.py`. No completed drawing-motion script is supplied.

The provided converter `scripts/image_to_targets.py` extracts OpenCV contour
**boundaries** and maps selected pixel points into a physical rectangle of user-specified
width, centred at a user-specified world position. **No A4 or paper model.**

Image frame: `image_center` (origin at the centre of the **full raster**, X right,
Y up, Z normal to the image). Default image width: 0.18 m; image height follows
pixel aspect ratio. World centre defaults to `(0.55, 0.0, 0.46)` m and can be
changed. These values are for mock simulation, not real workcell calibration.

Example:

```bash
ros2 run mas248_ur5e_demos image_to_targets pumpkin_outline.jpg \
  --width-m 0.18 --center-x 0.55 --center-y 0 --center-z 0.46 \
  --max-contours 4 --max-points 40 --open-stages --rviz
```

This writes `/tmp/mas248_targets.json` with `strokes[].poses` in the world
frame and `strokes[].poses_image` in the image-centred frame. The restored
`show_targets_rviz.py` preview publishes contour lines and optional TCP axes
in `world` on `/mas248/drawing_targets` (MarkerArray). Set the RViz fixed frame
to `world`. This preview script is a replacement for the module missing from
the lab backup.

For split launches, scripts, tests and student assignment, see the complete
`MAS248_L9_Student_Image_Frame_Guide.txt` included alongside `src/` in ZIP.

From this directory, run offline tests:

```bash
python3 -m unittest discover -s tests -v
```

These tests do not establish reachability, collision-free planning, actual
pen-stroke centreline paths, or physical robot safety.
