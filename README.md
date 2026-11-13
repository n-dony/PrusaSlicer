![PrusaSlicer logo](/resources/icons/PrusaSlicer_128px.png)

# PrusaSlicer

**A fork of PrusaSlicer for people who want to print fast on thick layers without giving up surface quality or strength.**

The fork has three goals:

1. **Speed without the usual penalty.** Print thick layers where nobody looks (interior walls, infill) and keep fine detail where people do (the visible wall, bridges, overhangs).
2. **Control.** Decide where seams go and how perimeters are ordered, laid down, cooled and heated.
3. **Multi-pass as a feature, not a hack.** Two-pass bridges, high-definition outer walls and combined internal perimeters are tunable options with a GUI, not manual G-code tricks.

## What this branch adds

This branch is **PrusaSlicer 3.0.0-alpha14 + the first-generation fork features**. The newer features (new combine engine, combine-aware cooling, high-definition outer wall, bridge scope and crosshatch, rotating seam, opposite-side inner seams) are in the **2.9.7 branch** ([`master`](https://github.com/n-dony/PrusaSlicer/tree/master)) and are being ported here. See "Not in this branch yet" below.

Where to find things: print settings, category **Walls & perimeters**, group **Walls quality**, unless noted.

| Feature | What it does | Why you want it | Where / option names |
|---|---|---|---|
| First / second internal perimeter roles | The first and second internal perimeters are distinct extrusion roles, each with its own speed and acceleration (and temperature offset, see below). | Tune the walls just behind the visible surface separately from deep inner walls. | Speed and acceleration settings: `first_internal_perimeter_speed`, `second_internal_perimeter_speed`, `first_internal_perimeter_acceleration`, `second_internal_perimeter_acceleration` |
| Perimeter order | Reverse the order of internal perimeters from a chosen depth. Optionally swap the first internal perimeter with the external one, so the external wall is laid first and the first internal perimeter is printed after it. | Control which wall is printed against which, for surface finish and dimensional accuracy. | `reverse_internal_perimeters` ("Reverse internal perimeters"), `reverse_internal_perimeters_at` ("Reverse internal perimeters at", default 2), `swap_first_int_w_ext_perimeter` ("Swap first internal with external perimeter") |
| Combine perimeters across layers | Print the first internal perimeter and the deeper internal perimeters every N layers, as thicker walls. The external wall is never combined. | Fewer, thicker internal walls print much faster; the visible wall stays at full resolution. | Category **Infill**, group **Infill combination**: `first_internal_perimeter_every_layers`, `second_internal_perimeter_every_layers` (1 = off) |
| Per-role temperature offsets | 12 independent offsets of +-50 C, one per role (external, first/second internal, perimeter, overhang perimeter, infill, solid infill, top solid infill, bridge, gap fill, support material, ironing). | Hotter for bridges and slow outer walls, cooler for fast infill, in the same layer. | Category **Temperature offsets**: `enable_temperature_offsets`, `temperature_offset_layers`, `*_temperature_offset` (e.g. `bridge_temperature_offset`) |
| Two-pass bridges (first version) | A bridge is laid in two half-height passes: a foundation, then the final pass. A simple on/off here. *Experimental.* | Cleaner, better-supported bridges on thick layers. | `two_pass_bridge` ("Two-pass bridge"; becomes the multi-choice `two_pass_bridge_scope` in master), `bridge_anchor_length` |
| High definition print | A checkbox that forces a quality-oriented bundle: at least 3 perimeters, first internal perimeter swapped, first/second internal perimeters and infill combined every 2 layers, scarf seam on contours. | One checkbox for a quality-oriented setup. | `high_def_print` ("High definition print") |

**Caution with `high_def_print`.** While it is ticked it re-applies those values, so the settings it forces are overwritten whenever it runs. Untick it before tuning the individual options. In master it is replaced by a one-shot button.

### Perimeter order, explained

Three internal perimeters, `E` = external, `1`/`2`/`3` = first/second/third internal. Default order, and the order with the swap:

```
 default                      swap_first_int_w_ext_perimeter
 print order: E, 1, 2, 3      print order: E, 2, 3, 1
 (the first internal wall     (the first internal wall closes the
  follows the outer wall)      groove behind the outer wall, last)
```

`reverse_internal_perimeters` with `reverse_internal_perimeters_at` = 2 reverses the order of the internal perimeters from depth 2, and can be combined with the swap.

### Combine perimeters, explained

With `first_internal_perimeter_every_layers` = 2 and `layer_height` 0.2, the first internal perimeter is printed once per two layers at double height. The external wall keeps printing every layer.

```
 every_layers = 1             every_layers = 2
 layer 2  E 1 2 3             layer 2  E      (internal walls skipped)
 layer 1  E 1 2 3             layer 1  E 1 2 3 (internal walls at 0.4 mm)
 (everything 0.2 mm)
```

Combined layers change the print time of a layer. This branch has no combine-aware cooling yet, so check the fan and minimum layer time settings on short layers.

## Not in this branch yet

These exist in the 2.9.7 branch only, and need a per-feature port to v3:

* new combine engine (per-loop matching, automatic depth, several methods) and combine-aware cooling
* high-definition outer wall (`external_perimeter_sublayers`) and sub-pass seams
* two-pass bridge scope and crosshatch variants
* rotating seam and opposite-side inner seams
* Quality preset as a one-shot button

## Quick start recipes

Starting points only. Tune for your printer, material and nozzle.

1. **Faster walls.** Use 3 perimeters, set `first_internal_perimeter_every_layers` and `second_internal_perimeter_every_layers` to 2. The external wall stays at full resolution.
2. **Better outer surface.** Enable `swap_first_int_w_ext_perimeter`, so the first internal perimeter is printed after the outer wall.
3. **Cleaner bridges.** Tick `two_pass_bridge` and check the first bridge in the preview. Experimental.
4. **Hotter bridges, cooler infill.** Enable `enable_temperature_offsets`, set `bridge_temperature_offset` to +10 and `infill_temperature_offset` to -5.

## Try it

The fork lives at <https://github.com/n-dony/PrusaSlicer>.

| Branch | Contents |
|---|---|
| `master` | PrusaSlicer 2.9.7 + all fork features (the most complete set) |
| `port/v3-fork-features-onto-alpha14` | PrusaSlicer 3.0.0-alpha14 + the ported first-generation features (this branch) |

**Build.** See the upstream [build documentation](doc/Build.md) for dependencies and steps.

**Gentoo.** Live ebuilds are available in the overlay: `media-gfx/prusaslicer-2.9999` (2.9.7 branch) and `media-gfx/prusaslicer-3.9999` (v3 branch).

## Status and honesty

* This is an **experimental community fork**. It is not made, supported or endorsed by Prusa Research. Please do not report fork problems to the upstream tracker.
* **Everything new is opt-in.** The defaults are meant to print like upstream, and presets from upstream load unchanged. If you see a difference with default settings, that is a bug: please report it.
* G-code written by the fork is standard G-code for your printer's firmware. Only the PrusaSlicer preview needs the fork to show the extra roles.
* **Preset compatibility.** Fork-only option keys are dropped (and replaced by defaults) when a preset or 3MF is opened in upstream PrusaSlicer.
* Features marked *experimental* are exactly that. Test on a cheap print before trusting them on a long one.
* Licensed under the **GNU AGPLv3**, like upstream.

## How to help

Try the options on your own models and compare with upstream prints of the same model. Open an issue at [n-dony/PrusaSlicer](https://github.com/n-dony/PrusaSlicer/issues) with the project **3MF** and a short **G-code snippet** around the problem (a few layers are enough), plus a photo if it is about surface quality. "It worked well on printer X with settings Y" reports are just as useful.

---

## About PrusaSlicer (upstream)

PrusaSlicer enables you to take your 3D models, generate 3D printing instructions and send them to your 3D printer. It supports both FDM 3D printers and mSLA 3D printers. It is developed by [Prusa Research](https://www.prusa3d.com/) and apart from Prusa printers it supports machines from a wide variety of manufacturers.

PrusaSlicer is originally based on [Slic3r](https://github.com/Slic3r/Slic3r) by Alessandro Ranellucci and the RepRap community.

## Installation

The **recommended installation method** is to go to the [PrusaSlicer project page](https://www.prusa3d.com/prusaslicer/) and download and install the software by following the instructions there.

**Alternatively**, for Windows and macOS, you can download the software directly from the [GitHub releases page](https://github.com/prusa3d/PrusaSlicer/releases). For Linux, PrusaSlicer is currently distributed exclusively through [Flathub](https://flathub.org/en/apps/com.prusa3d.PrusaSlicer).

If you prefer, you can always build PrusaSlicer yourself from source. See the [documentation](doc/Build.md) to learn how to do it.

### Main features

* Set the **printing parameters** with precision - from settings affecting the **whole print** down to **single-layer adjustments**.
* Modify your objects before printing to best suit your needs, you can **paint**, **cut**, **arrange** and do **many more** directly in PrusaSlicer.
* Open **multiple projects** at once, each containing **multiple beds** (each with potentially **different settings**).
* Make full use of **multi-material printing**
* Use your **preferred method to send your prints to the printer**, PrusaSlicer supports a wide range of possibilities.
* **View the generated print instructions** in an advanced **3D preview**.
* You can make use of the **command-line interface** to use **PrusaSlicer without GUI** in your automation setups.
* If you want, you **can make use of the integration** with [PrusaConnect](https://connect.prusa3d.com/) and [Printables](https://www.printables.com) to greatly simplify your workflow.

### Reporting a bug

Did you find a bug? Bugs can be reported in our [GitHub issue tracker](https://github.com/prusa3d/PrusaSlicer/issues), but **first make sure your report complies with the [Issue tracker policy](https://github.com/prusa3d/PrusaSlicer?tab=contributing-ov-file#issue-tracker-policy)**. **Are you not sure?** Get in touch in our [GitHub Discussions](https://www.github.com/prusa3d/discussions), before filing a bug report in the issue tracker. You can always file a bug report later, once you have more confidence.

### How to get in touch

We maintain a [GitHub Discussions](https://www.github.com/prusa3d/discussions) page in this repository to be used for **general discussions**, **questions** and **feature requests**. Furthermore, there is an announcements category, which we use to communicate with you directly.

### Pull requests

Read our [contribution guide](.github/CONTRIBUTING.md) to get more information.

### Technical stack

All of PrusaSlicer is written in C++, using CMake as the build system. The code assumes that the compiler supports C++20.

The slicing backend heavily relies on [Clipper](https://www.angusj.com/clipper2) by Angus Johnson, which handles polygon boolean operations, offsets and similar. [Eigen](https://libeigen.gitlab.io/) library is used for basic types and linear algebra calculations.

[wxWidgets](https://wxwidgets.org) library is used to handle GUI windows and events across platforms. Most of the UI is implemented using a custom OpenGL-based UI toolkit based on the [Yoga layout engine](https://github.com/facebook/yoga) and [Dear ImGui](https://github.com/ocornut/imgui), which makes it platform-independent.

The application uses many other libraries. You can see the [deps/](deps/) and [bundled_deps/](bundled_deps/) folders in the source tree to see the complete list. We are grateful to the authors and maintainers for open-sourcing their work.
