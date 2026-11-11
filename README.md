![PrusaSlicer logo](/resources/icons/PrusaSlicer_128px.png)

# PrusaSlicer

**A fork of PrusaSlicer for people who want to print fast on thick layers without giving up surface quality or strength.**

The fork has three goals:

1. **Speed without the usual penalty.** Print thick layers where nobody looks (interior walls, infill) and keep fine detail where people do (the visible wall, bridges, overhangs).
2. **Control.** Decide where seams go and how perimeters are ordered, laid down, cooled and heated.
3. **Multi-pass as a feature, not a hack.** Two-pass bridges, high-definition outer walls and combined internal perimeters are tunable options with a GUI, not manual G-code tricks.

## What this branch contains

This branch is **PrusaSlicer 3.0.0-alpha14 + the first-generation fork features**.

> The newer 2.9.x features (new combine engine, combine-aware cooling, high-definition outer wall, bridge scope/crosshatch, seam work) live in the **2.9.7 branch** ([`master`](https://github.com/n-dony/PrusaSlicer/tree/master)) and are being ported to v3.

| Feature | What it does | Why you want it | Option names |
|---|---|---|---|
| First / second internal perimeter roles | Distinct extrusion roles with their own speed, acceleration and print-order controls. | Tune the walls just behind the visible surface separately from deep inner walls. | Print settings (speed, acceleration, walls); order options below |
| Perimeter order | Reverse internal perimeters from a chosen depth; print the first internal perimeter before the external one. | Control surface finish and dimensional accuracy. | `reverse_internal_perimeters`, `reverse_internal_perimeters_at`, `swap_first_int_w_ext_perimeter` |
| Per-role temperature offsets | 12 independent offsets of +-50 C, one per role, applied during extrusion. | Hotter for bridges and slow walls, cooler for fast infill. | `enable_temperature_offsets`, `temperature_offset_layers`, `*_temperature_offset` (e.g. `bridge_temperature_offset`) |
| Two-pass bridges (first version) | Bridges laid in two passes, foundation then final. *Experimental*; a simple on/off in this branch. | Better-supported bridges on thick layers. | `two_pass_bridge` (renamed to `two_pass_bridge_scope` in master), `bridge_anchor_length` |
| High definition print | Quality preset switch. | One checkbox for a quality-oriented setup. | `high_def_print` ("High definition print") |

In this branch, per-role perimeter combining is not exposed as its own option keys; the dedicated combine options of the 2.9.7 branch are not ported yet.

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
