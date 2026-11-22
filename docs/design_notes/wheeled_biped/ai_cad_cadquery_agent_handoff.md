# AI CAD and CadQuery robotics — agent handoff

Prepared: 2026-11-22. Research summarized from this conversation, primarily performed on 2026-11-20. This document preserves findings; it is not a fresh audit of every linked project.

## 1. User goal and context

The user works on robotics software, ROS 2, control, MPC and state estimation, but is unfamiliar with mechanical design. They want AI assistance to create editable CAD, especially robotic arms, legged robots, motor mounts, actuators and gearboxes. They prefer a programming-friendly workflow and asked specifically about editing and rendering CadQuery in VS Code.

No particular mechanism, load case, manufacturing process, operating system or target dimensions have been selected. Do not assume the user wants a complete robot fabricated yet.

## 2. Current findings and verification boundary

- Recommended code workflow: AI coding assistant → CadQuery Python → OCP CAD Viewer in VS Code → editable Python source plus STEP export; STL when needed for printing.
- Recommended conversational CAD option: Zoo Design Studio with Zookeeper.
- Adam is another option when working inside an existing CAD application.
- The more established repositories found were gear and standard-component libraries. Complete CadQuery robot repositories had much smaller communities.
- Prior research inspected documentation and GitHub pages. No CAD packages were installed, no repositories cloned, no examples executed, and no geometry or physical performance independently validated.
- Star counts below are historical approximate snapshots, not quality ratings or current guarantees.
- Claims about geometry checks and simulations are the authors' descriptions unless explicitly stated otherwise.

## 3. AI CAD tools

| Tool | Role | Why consider it | Limits / next checks |
| --- | --- | --- | --- |
| [Zoo Design Studio / Zookeeper](https://zoo.dev/) | Conversational generation and editing of parametric CAD | Beginner-oriented text iteration; native KCL model; STEP/STL export | Retain native project/KCL. Check current capabilities, pricing and assembly requirements before committing. |
| [Adam](https://adam.new/) | AI assistance through connected CAD applications | Useful for creating or changing parts in an existing CAD environment, including Onshape/Fusion workflows | Check the specific application integration and access requirements. Its engineering claims were not independently tested here. |
| [CadQuery](https://cadquery.readthedocs.io/en/stable/intro.html) + an AI coding assistant | AI writes parametric Python that constructs solid geometry | Dimensions as variables, Git versioning, reusable functions, scripted checks and export | Requires a Python environment and code execution. A valid solid does not establish structural adequacy or manufacturability. |

Primary references:

- [Zoo Zookeeper workflow](https://zoo.dev/docs/zoo-design-studio/zookeeper)
- [Zoo installation/getting started](https://zoo.dev/docs/zoo-design-studio/getting-started)
- [Zoo exports](https://zoo.dev/docs/zoo-design-studio/features/data-management/export)
- [Adam CAD modeling workflow](https://docs.adam.new/cad/modeling)
- [Adam supported software](https://docs.adam.new/supported-software)
- [CadQuery import/export](https://cadquery.readthedocs.io/en/latest/importexport.html)

Adjacent options surfaced during research, but were not the main recommendations: Autodesk Assistant in Fusion and Onshape's FeatureScript MCP workflow. Check current access and exact capabilities before recommending them. Onshape AI Advisor's documentation describes guidance/help; do not equate that alone with autonomous CAD generation.

### File formats

- Keep the parametric source: CadQuery `.py`, Zoo native/KCL, or native CAD document.
- STEP carries precise exchange geometry, but generally not the original sketches, parameters and feature history.
- STL is a triangulated mesh for workflows such as printing; it is not the editable parametric master.
- If generating robot descriptions, explicitly handle units: this proposed CAD workflow uses millimeters; URDF link positions and mesh scale need consistent meters.

## 4. VS Code editing and 3D preview

Use the community extension **OCP CAD Viewer**, by bernhard-42. It supports both CadQuery and build123d; the repositories listed below should still be distinguished by which CAD library they actually use.

- [VS Code Marketplace](https://marketplace.visualstudio.com/items?itemName=bernhard-42.ocp-cad-viewer)
- [Maintainer repository](https://github.com/bernhard-42/vscode-ocp-cad-viewer)
- [Installation documentation](https://bernhard-42.github.io/ocp_viewer_docs/viewers/ocp_vscode/installation/)
- [Overview](https://bernhard-42.github.io/ocp_viewer_docs/viewers/ocp_vscode/overview/)

Suggested setup:

1. Install VS Code's Microsoft Python extension and OCP CAD Viewer.
2. Create/select a dedicated Python environment with **Python: Select Interpreter**.
3. Use the viewer sidebar's **Quickstart CadQuery** to install the CAD libraries and companion Python package.
4. Keep the extension and `ocp_vscode` Python package at compatible versions; use the extension's installer if unsure.
5. Open a Python model, open/start the viewer, then use **Run → Run Without Debugging**. Each `show(...)` call sends geometry to the viewer.
6. Change dimensions and rerun. Do not promise automatic rebuild-on-save unless separately configured and tested.

The viewer offers rotation/pan/zoom, geometry inspection, measurement tools and visual debugging. Geometry edits remain in Python; this is not a conventional sketch-dragging CAD editor.

Minimal starter script (provided as an example, not executed in this session):

```python
from pathlib import Path
import cadquery as cq
from ocp_vscode import show

# All CAD dimensions in mm.
length, width, thickness = 80.0, 40.0, 5.0
hole_diameter, hole_spacing = 4.5, 60.0

plate = (
    cq.Workplane("XY")
    .box(length, width, thickness)
    .faces(">Z").workplane()
    .pushPoints([(-hole_spacing / 2, 0), (hole_spacing / 2, 0)])
    .hole(hole_diameter)
)

show(plate)
output = Path(__file__).resolve().parent / "exports"
output.mkdir(exist_ok=True)
cq.exporters.export(plate, str(output / "plate.step"))
```

For repeatable automation, separate geometry-building functions from viewer calls and export entry points, so headless builds do not require a running viewer.

## 5. Open-source repositories found

### 5.1 Gear and component libraries — best established starting points

| Repository | Contents | Prior popularity snapshot | Main caveat |
| --- | --- | --- | --- |
| [meadiode/cq_gears](https://github.com/meadiode/cq_gears) | Involute spur, helical, herringbone, ring and bevel gears; planetary gearsets; racks; example notebooks | ~169 stars | README labels it work in progress. Its old CadQuery version notes need checking against a tested current environment. |
| [gumyr/cq_warehouse](https://github.com/gumyr/cq_warehouse) | Bearings, fasteners, threads, chains, sprockets; matching hole tools and assembly helpers | ~155 stars | Reusable parts library, not a complete actuator or robot design. Check compatibility with the selected CadQuery version. |

Suggested use: generate transmission geometry with `cq_gears`, use standard hardware from `cq_warehouse`, and model housings, shafts, supports and mounting interfaces explicitly. Gear geometry generation does not establish tooth strength, backlash, efficiency, thermal capacity or fatigue life.

Useful starting locations:

- `cq_gears/examples/`
- [cq_warehouse documentation](https://cq-warehouse.readthedocs.io/en/latest/) — includes a bearing pillow-block example, clearance holes, bearings and BOM helpers.

### 5.2 Robot arms and legged robots — smaller examples

| Repository | What was found | Prior popularity snapshot | Best reason to inspect |
| --- | --- | --- | --- |
| [yotown/gym](https://github.com/yotown/gym) | CadQuery reconstructions of printed parts from SO-101, Open Duck Mini v2 and LeRobot humanoid; Flamingo variant; shared parameters, STEP/STL export and geometry checks | ~9 stars | Most relevant cross-section of arm and legged-robot parts as Python. These are third-party reconstructions, not proof that upstream native CAD was CadQuery. |
| [Danion1/robot-arm-moveit2](https://github.com/Danion1/robot-arm-moveit2) | 4-DoF DYNAMIXEL arm and parallel gripper; CadQuery → meshes/mass/inertia/URDF → Gazebo + ros2_control + MoveIt 2 | 0 stars | Particularly relevant to the user's ROS 2 interests. Author explicitly says simulation only, not yet printed. |
| [nbaron17/quadruped-cadquery](https://github.com/nbaron17/quadruped-cadquery) | 12-DoF MG996R quadruped; chassis, shoulders, hip links, upper/lower legs, feet; assembly, BOM and exports | ~1 star, 2 commits | Small modular example for learning part organization. Do not infer dynamic locomotion or hardware validation from a CAD model. |

Entry points for the next agent:

**yotown/gym**

- `robots/so101/shared.py`: common dimensions/interfaces.
- `robots/so101/parts/upper_arm.py`: individual part example.
- Inspect Open Duck Mini v2 parts for leg mechanisms.
- README documents surface deviations from original models; verify critical interfaces independently.
- Documented commands: `pip install -e .`, `gym check so101`, `gym build so101 --step --stl`, `gym diff so101`.
- Its `gym` CLI is project-specific; do not confuse it with the unrelated OpenAI Gym package.
- Check per-robot upstream licenses and notices, not only the top-level license.

**robot-arm-moveit2**

- `cad/`: parts, assemblies, mass/inertia and robot-description generation/checks.
- `cad/lib/`: shared CAD helpers copied into the repository.
- `cad_export/`: exported printable CAD.
- `ros2/`: description, controller and MoveIt packages.
- README reports ROS 2 Jazzy, Gazebo Sim 8 and Python 3.12. Some code identifiers/comments are Portuguese.
- Distinguish author's reported simulation checks from independent validation by the resuming agent.

**quadruped-cadquery**

- `robot/config.py`, `robot/assembly.py`, `robot/export_parts.py`, `robot/bom.py`.
- `view_assembly.py` is documented for CQ-editor; adapt visualization to OCP CAD Viewer after inspecting how the assembly is exposed.
- Documented exports: `python -m robot.export_parts`.

### 5.3 Cycloidal reducer example

[tscircuit/cycloidal-gearbox](https://github.com/tscircuit/cycloidal-gearbox) — approximately 0 stars in the prior snapshot.

- CG58: nominal 19:1 dual-disc cycloidal cartridge, 20 ring pins, 19-lobe discs, six output pins and 44 separate solids.
- Main files: `cycloidal.py`, `gearbox_adapter.py`, `validate.py`, `explode.py`.
- STEP export, animated/exploded assembly explorer and author-described geometry checks.
- Supported customization focuses on NEMA14/17 mounting and output adapters; do not assume ratio/eccentricity can be changed without redesign and validation.
- Bearings are simplified envelopes and the motor is an external interface, not a full internal motor model.
- README explicitly leaves threads, manufacturing fits, lubrication, shaft/pin retention, preload and bearing retention to fabrication engineering. It does not claim qualified torque, backlash or service life.

This is a mechanism/geometry reference, not a validated actuator ready for production.

### Search gaps and exclusions

No comparably established CadQuery BLDC electromagnetic motor-design repository was identified. Do not present mounting geometry or a gearbox as an electromagnetic motor design tool. Search results using build123d or providing only STEP/STL were not treated as confirmed CadQuery source repositories.

## 6. Recommended continuation

The following is proposed next work; it has not been performed:

1. Establish the user's intended first part/mechanism and target platform. A bracket, bearing support or small gear assembly is a manageable first exercise.
2. Set up CadQuery plus OCP CAD Viewer and verify a minimal model renders.
3. Clone the chosen repository; inspect its README, license, dependencies and any AGENTS.md before running code. Record commit SHA and package versions.
4. Run one existing example unchanged. Record actual failures rather than silently rewriting the project around them.
5. Expose meaningful parameters and preserve purchased-hardware interfaces. Modify one dimension and verify the resulting change.
6. Export native Python source, STEP and a preview. Check valid solids, dimensions, hole patterns, body count and assembly placement. Label checks as geometric, simulation-based or physical.
7. For ROS 2 use, verify joint axes/origins, units, mesh transforms, collision geometry and density-based mass/inertia conventions.
8. Before fabrication, determine material/process, fits, fastening, assembly access and load cases. Avoid claiming strength based solely on a successful CAD rebuild.

## 7. Reusable design prompt

```text
I understand robotics and programming but am a beginner in mechanical design.
Help me design [part/function] using CadQuery and preview it with OCP CAD Viewer
in VS Code.

Known hardware/drawings: [...]
Envelope and mounting interfaces: [...]
Payload, forces, torques and duty cycle: [...]
Motion and clearance requirements: [...]
Manufacturing method/material: [known or undecided]

First identify missing information that affects fit or strength. Explain a simple
design, list assumptions, and do not invent purchased-hardware dimensions.
Use mm, named parameters, standard fasteners and readable functions.
Preserve mating interfaces during revisions. Explain assembly access and process
constraints. Separate geometry generation from optional viewer/export calls.

Return runnable Python, STEP, a preview and STL if needed. Verify what you can:
solid validity, dimensions, holes, body count and relevant interference checks.
State which checks were actually executed and what remains unverified.
Do not describe the result as load-qualified or fabrication-ready without evidence.
```

## 8. Copy-paste resume instruction

```text
Read this handoff and continue the AI-assisted CadQuery robotics workflow.
Research is complete enough to choose initial examples; installations and builds
have not been tested. Start with an isolated Python environment and a minimal
CadQuery + OCP CAD Viewer model. For gears inspect meadiode/cq_gears; for robot
parts inspect yotown/gym; for CAD-to-ROS 2 inspect Danion1/robot-arm-moveit2.
Record versions/commits, run an unchanged example first, and report evidence.
Keep editable Python plus STEP. Distinguish valid geometry, simulated behavior
and physically validated mechanics. Obtain missing target-part requirements
before making consequential mechanical assumptions.
```
