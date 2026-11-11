
![PrusaSlicer logo](/resources/icons/PrusaSlicer.png?raw=true)

# PrusaSlicer

**A fork of PrusaSlicer for people who want to print fast on thick layers without giving up surface quality or strength.**

The fork has three goals:

1. **Speed without the usual penalty.** Print thick layers where nobody looks (interior walls, infill) and keep fine detail where people do (the visible wall, bridges, overhangs).
2. **Control.** Decide where seams go and how perimeters are ordered, laid down, cooled and heated.
3. **Multi-pass as a feature, not a hack.** Two-pass bridges, high-definition outer walls and combined internal perimeters are tunable options with a GUI, not manual G-code tricks.

## What the fork adds

Where to find things: all options below are on the print settings page **Layers and perimeters** unless noted. Mode in brackets is the minimum mode that shows the option (Simple / Advanced / Expert).

| Feature | What it does | Why you want it | Where / option names |
|---|---|---|---|
| First / second internal perimeter roles | The first and second internal perimeters are distinct extrusion roles, each with its own speed, acceleration, print-order controls and preview colour. | Tune the wall layers that matter (just behind the visible surface) separately from deep inner walls. | Print settings pages **Speed** and **Advanced** (e.g. `first_internal_perimeter_speed`, `first_internal_perimeter_acceleration`); preview feature types |
| Perimeter order | Reverse the order of internal perimeters from a chosen depth, or print the first internal perimeter last within its group (after the external wall in the default inside-out order). | Control which wall is printed against which, for surface finish and dimensional accuracy. | Group **Advanced** (all Expert): `reverse_internal_perimeters` ("Reverse internal perimeter order"), `reverse_internal_perimeters_at` ("Reverse internal perimeters starting at"), `first_internal_perimeter_last` ("Print first internal perimeter last") |
| Per-role temperature offsets | 13 independent offsets of +-50 C, one per role (external, first/second internal, perimeter, overhang, infill, solid, top solid, bridge, gap fill, support, support interface, ironing). | Hotter for bridges and slow outer walls, cooler for fast infill, in the same layer. | Print settings page **Temperature offsets** (Expert): `enable_temperature_offsets`, `temperature_offset_layers`, `*_temperature_offset` (e.g. `bridge_temperature_offset`) |
| Two-pass bridges | A bridge is laid in two half-height passes: a foundation, then the final pass. Automatically skipped when half the layer height would be below the printer's minimum layer height. *Experimental.* | Cleaner, better-supported bridges and overhang perimeters on thick layers. | Group **Quality (slower slicing)** (Advanced): `two_pass_bridge_scope` ("Two-pass bridges": disabled / bridge infill only / bridge infill + perimeters / crosshatch variants), `bridge_anchor_length` ("Two-pass bridge anchor length") |
| Combine perimeters across layers | Print internal perimeters every N layers as thicker walls. Per-loop matching with selectable methods, plus an automatic mode that picks the combination depth. Internal solid infill combining is coupled to the groups. **The external wall is never combined.** | Fewer, thicker internal walls print much faster; the visible wall stays at full resolution. | Group **Combine perimeters**: `automatic_internal_perimeters_combination` (Advanced), `combine_perimeters_max_shift` (Advanced), `combine_perimeters_method` (Expert), `combine_perimeters_min_arc` (Expert), `combine_perimeters_atomic_fragments` (Expert). Methods: "Whole loops (strict)", "Whole loops (tolerant)", "Partial arcs", and "Legacy A/B/C" (older algorithms, kept for comparison) |
| Combine-aware cooling | Cooling treats a combined group as one unit: group time or top-layer ceiling, real group tops, fan follows the layer. | Combining layers does not wreck minimum-layer-time cooling. | Filament settings, page **Cooling** (Expert): `cooling_combine_logic` ("Cooling of combined layers": Whole group time / Top layer sets speed) |
| High-definition outer wall | Thick layers, but the outer wall is laid as k high-definition sub-layers. See below. | Fine-layer surface on a thick-layer print. | Group **Quality (slower slicing)** (Expert): `external_perimeter_sublayers` ("External perimeter sub-layers", 1 to 8, default 1 = off); Arachne only |
| Quality preset | A button that applies a bundle of settings suited to high-definition printing once. It never restores the old values. | One click for a quality-oriented setup. | Group **Quality preset** (Expert): button "Apply high definition preset" (`high_def_print`) |
| Preview | Logarithmic speed colour views ("Speed (log) (mm/s)", "Actual speed (log) (mm/s)"). | Speed differences between 20 and 300 mm/s stay readable. | G-code preview view type |

### High-definition outer wall

The interior prints thick and fast; only the visible wall is split into thinner sub-layers. Example: `layer_height` 0.3 mm with `external_perimeter_sublayers` 3.

```
 Thick layer (sublayers 1)         Sublayers 3
 +----------+---------------+      +----+------------+
 |          |               |      | .1 |            |
 | outer    |   interior    |      +----+  interior  |   outer wall: 3 x 0.1 mm
 | wall     |  (0.3 mm)     |      | .1 |  (0.3 mm)  |   interior:   1 x 0.3 mm
 | (0.3 mm) |               |      +----+            |
 |          |               |      | .1 |            |
 +----------+---------------+      +----+------------+
```

Sub-passes are real G-code levels. Their fan and cooling follow the layer group, and their seams follow the layer's seam in all seam modes, scarf included. Arachne perimeters only.

### In development (coming in the next release)

These two are implemented in the working tree but not yet committed or released, so they are not part of the supported feature set yet. Both live in the group **Seam** (Advanced).

* **Rotating seam** (`seam_position` = "Rotating"): the seam moves by a fixed angle every layer so seams form a deliberate pattern instead of one visible line. Options: "Seam rotation defined by" (`seam_rotation_define_by`), "Number of seam positions" (`seam_rotation_count`), "Angle between seam positions" (`seam_rotation_step`), "Seam rotation start angle" (`seam_rotation_start`), "Seam rotation order" (`seam_rotation_order`: Spread / Sequential), "Seam rotation drift" (`seam_rotation_drift`), "Seam on holes" (`seam_rotation_hole_offset`), "Advance seam per" (`seam_rotation_sublayers`). Example: 4 positions with order Spread. Top view of a cylinder, numbers are the position used by layer 1, 2, 3, 4 (then it repeats):

```
 Seen from above: +X to the right (east), +Y up (north), angles counter-clockwise from +X.

                   90 deg
                  [3] (layer 3)
              .-----------.
             /             \
 180 deg [2]      top        [1] 0 deg
              \             /
               '-----------'
                  [4] (layer 4)
                   270 deg

 Spread order (N = 4): layer 1 = 0, layer 2 = 180, layer 3 = 90, layer 4 = 270 degrees
 (consecutive layers use positions far apart; "Sequential" would go 0, 90, 180, 270)
```

  The seams end up in four vertical columns.

* **Opposite-side inner seams** (`staggered_inner_seams` on, `staggered_inner_seams_mode` = "Opposite side (half loop)", value `half_loop`): inner loops alternate between the external seam position and the opposite side, half a loop away, by perimeter depth parity. Three nested loops (P0 external, P1, P2), `*` marks where each loop starts:

```
 P0 *---------------+      P1 +---------------+      P2 *---------------+
    |               |         |  .-----------.|         |               |
    |               |         |  |           ||         |               |
    +---------------+         +--+-----------*+         +---------------+
   (starts top-left)       (starts bottom-right)       (starts top-left)
```

## Quick start recipes

Starting points only. Tune for your printer, material and nozzle.

1. **Fast thick print with a fine outer wall.** Use Arachne perimeters, set `layer_height` to 0.3 and `external_perimeter_sublayers` to 3. Interior prints at 0.3 mm, the visible wall at 3 x 0.1 mm.
2. **Cleaner bridges.** Set `two_pass_bridge_scope` to "Bridge infill + perimeters". Optionally set `bridge_anchor_length` so the perimeter passes anchor into neighbouring walls. Mark this as experimental and check the first bridge in the preview.
3. **Stronger, faster walls.** Enable `automatic_internal_perimeters_combination` (or choose a `combine_perimeters_method` and combine every N layers) and set `cooling_combine_logic` to "Whole group time". The external wall stays uncombined.

## Try it

The fork lives at <https://github.com/n-dony/PrusaSlicer>.

| Branch | Contents |
|---|---|
| `master` | PrusaSlicer 2.9.7 + all fork features |
| `port/v3-fork-features-onto-alpha14` | PrusaSlicer 3.0.0-alpha14 + the first-generation fork features (see that branch's README) |

**Build.** Follow the upstream guides for your platform, which list the dependencies: [Linux](doc/How%20to%20build%20-%20Linux%20et%20al.md), [macOS](doc/How%20to%20build%20-%20Mac%20OS.md), [Windows](doc/How%20to%20build%20-%20Windows.md). The Linux guide's recipe, after building `deps/`, is:

```
mkdir build && cd build
cmake .. -DSLIC3R_STATIC=1 -DSLIC3R_GTK=3 -DSLIC3R_PCH=OFF \
         -DCMAKE_PREFIX_PATH=$(pwd)/../deps/build/destdir/usr/local
make -j4
```

**Gentoo.** Live ebuilds are available in the overlay: `media-gfx/prusaslicer-2.9999` (2.9.7 branch) and `media-gfx/prusaslicer-3.9999` (v3 branch).

## Status and honesty

* This is an **experimental community fork**. It is not made, supported or endorsed by Prusa Research. Please do not report fork problems to the upstream tracker.
* **Everything new is opt-in.** The defaults are meant to print like upstream, and presets from upstream load unchanged. If you see a difference with default settings, that is a bug: please report it.
* G-code written by the fork is standard G-code for your printer's firmware. Only the PrusaSlicer preview needs the fork to show the extra roles.
* **Preset compatibility.** Fork-only option keys are dropped (and replaced by defaults) when a preset or 3MF is opened in upstream PrusaSlicer. Older fork presets and 3MFs that use the former key names `external_perimeter_resolution` and `swap_first_int_w_ext_perimeter` load correctly in the fork.
* Features marked *experimental* are exactly that. Test on a cheap print before trusting them on a long one.
* Licensed under the **GNU AGPLv3**, like upstream.

## How to help

Try the options on your own models and compare with upstream prints of the same model. Open an issue at [n-dony/PrusaSlicer](https://github.com/n-dony/PrusaSlicer/issues) with the project **3MF** and a short **G-code snippet** around the problem (a few layers are enough), plus a photo if it is about surface quality. "It worked well on printer X with settings Y" reports are just as useful.

---

## About PrusaSlicer (upstream)

You may want to check the [PrusaSlicer project page](https://www.prusa3d.com/prusaslicer/).
Prebuilt Windows, OSX and Linux binaries are available through the [git releases page](https://github.com/prusa3d/PrusaSlicer/releases) or from the [Prusa3D downloads page](https://www.prusa3d.com/drivers/). There are also [3rd party Linux builds available](https://github.com/prusa3d/PrusaSlicer/wiki/PrusaSlicer-on-Linux---binary-distributions).

PrusaSlicer takes 3D models (STL, OBJ, AMF) and converts them into G-code
instructions for FFF printers or PNG layers for mSLA 3D printers. It's
compatible with any modern printer based on the RepRap toolchain, including all
those based on the Marlin, Prusa, Sprinter and Repetier firmware. It also works
with Mach3, LinuxCNC and Machinekit controllers.

PrusaSlicer is based on [Slic3r](https://github.com/Slic3r/Slic3r) by Alessandro Ranellucci and the RepRap community.

See the [project homepage](https://www.prusa3d.com/slic3r-prusa-edition/) and
the [documentation directory](doc/) for more information.

### What language is it written in?

All user facing code is written in C++.
The slicing core is the `libslic3r` library, which can be built and used in a standalone way.
The command line interface is a thin wrapper over `libslic3r`.

### What are PrusaSlicer's main features?

Key features are:

* **multi-platform** (Linux/Mac/Win) and packaged as standalone-app with no dependencies required
* complete **command-line interface** to use it with no GUI
* multi-material **(multiple extruders)** object printing
* multiple G-code flavors supported (RepRap, Makerbot, Mach3, Machinekit etc.)
* ability to plate **multiple objects having distinct print settings**
* **multithread** processing
* **STL auto-repair** (tolerance for broken models)
* wide automated unit testing

Other major features are:

* combine infill every 'n' perimeters layer to speed up printing
* **3D preview** (including multi-material files)
* **multiple layer heights** in a single print
* **spiral vase** mode for bumpless vases
* fine-grained configuration of speed, acceleration, extrusion width
* several infill patterns including honeycomb, spirals, Hilbert curves
* support material, raft, brim, skirt
* **standby temperature** and automatic wiping for multi-extruder printing
* [customizable **G-code macros**](https://github.com/prusa3d/PrusaSlicer/wiki/PrusaSlicer-Macro-Language) and output filename with variable placeholders
* support for **post-processing scripts**
* **cooling logic** controlling fan speed and dynamic print speed

### Development

If you want to compile the source yourself, follow the instructions on one of
these documentation pages:
* [Linux](doc/How%20to%20build%20-%20Linux%20et%20al.md)
* [macOS](doc/How%20to%20build%20-%20Mac%20OS.md)
* [Windows](doc/How%20to%20build%20-%20Windows.md)

### Can I help?

Sure! You can do the following to find things that are available to help with:
* Add an [issue](https://github.com/prusa3d/PrusaSlicer/issues) to the github tracker if it isn't already present.
* Look at [issues labeled "volunteer needed"](https://github.com/prusa3d/PrusaSlicer/issues?utf8=%E2%9C%93&q=is%3Aopen+is%3Aissue+label%3A%22volunteer+needed%22)

### What's PrusaSlicer license?

PrusaSlicer is licensed under the _GNU Affero General Public License, version 3_.
The PrusaSlicer is originally based on Slic3r by Alessandro Ranellucci.

### How can I use PrusaSlicer from the command line?

Please refer to the [Command Line Interface](https://github.com/prusa3d/PrusaSlicer/wiki/Command-Line-Interface) wiki page.
