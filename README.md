# Real-Time GPU Surgical Soft-Tissue Simulation 

<div align="center">
  <p>A high-performance, real-time surgical cutting and soft-tissue simulation framework built with Unity and Compute Shaders.</p>
</div>

## 📌 Introduction

This project aims to provide a highly realistic and physically accurate simulation of soft-tissue (e.g., liver) deformation and surgical incision. Traditional soft-body cutting algorithms often suffer from topological inconsistencies (such as "lotus-root" bridging artifacts) and severe performance bottlenecks. 

By leveraging **Extended Position-Based Dynamics (XPBD)** fully accelerated on the GPU, combined with a robust, swept-surface continuous collision detection (CCD) and tetrahedral subdivision algorithm, this framework achieves seamless, real-time surgical cutting with surgical-grade topological stability.

## 🚀 Core Features

- **Real-Time GPU Physics Engine**: Custom-built `XPBDSolverGPU` using Unity Compute Shaders, handling thousands of tetrahedrons and particles in parallel without dropping frame rates.
- **Continuous Swept-Surface Cutting**: Implements dynamic plane tracking that responds to high-speed rotational tool movements (e.g., scalpel swiping or flicking).
- **Advanced Topological Management**: 
  - **Cross-Frame Consistency**: A dedicated stroke-level caching system (`_strokeVertexSide`) strictly tracks and locks topological associations, completely eliminating "lotus-root" (藕断丝连) bridging artifacts.
  - **Trajectory Correction (Snapping)**: Mathematically enforces clean cuts by snapping intersections to original vertices when below tolerance thresholds.
  - **Rigorous Collision Detection**: Uses vertex-level inclusion and rigid AABB-AABB bounding box intersections to prevent missing sliver (degenerate) tetrahedrons.
- **Dynamic Surface Reconstruction**: Instantaneous updating of the rendering mesh using internal barycentric subdivision mapping without stalling the physics loop.

## 🛠️ Technical Stack

- **Game Engine**: Unity 3D (C#)
- **Physics Architecture**: XPBD (Extended Position-Based Dynamics)
- **Parallel Computing**: Unity Compute Shaders (HLSL)
- **Mesh Processing**: Tetrahedral subdivision based on the latest computer graphics research (Inspired by PG2025: *"Parallel Constraint Graph Partitioning and Coloring for Realtime Soft-Body Cutting"*).

## 🧠 Key Algorithm: The Cutting Pipeline

The cutting mechanism, primarily located in `TetSubdivisionCutter.cs` and `CuttingToolV3.cs`, operates in the following robust pipeline:

1. **Spatial Filtering**: Conducts strict `AABB-AABB` testing between the swept volume of the scalpel and the active tetrahedrons.
2. **Topological Classification**: Evaluates the signed distance of tetrahedron vertices relative to the dynamically constructed cutting plane.
3. **Cross-Frame Locking**: Ensures that vertices do not artificially "flip" sides during rapid curved blade motions, maintaining strict manifold geometry.
4. **Subdivision (Case 1 & Case 2)**: Splits intersecting tetrahedra into 4 or 6 sub-tetrahedra respectively, automatically correcting winding orders based on rest positions to prevent inside-out normal flipping.
5. **GPU Flush**: The new topological constraints and particles are asynchronously flushed to the GPU `XPBDSolver` for continuous physics simulation.

## 📂 Project Structure (Key Components)

- `Assets/SurgicalSim/Physics/XPBDSolverGPU.cs` - The CPU-side manager for dispatching XPBD physics kernels.
- `Assets/SurgicalSim/Shaders/XPBDSolver.compute` - The core physics calculations (Solver, Integration, Collision) executed on the GPU.
- `Assets/SurgicalSim/CuttingV3/TetSubdivisionCutter.cs` - The heart of the topological mesh cutting mathematics.
- `Assets/SurgicalSim/CuttingV3/CuttingToolV3.cs` - The surgical tool controller handling spatial detection and triggering cutting frames.
- `Assets/StreamingAssets/liver3-HD.msh` - High-definition volumetric tetrahedral mesh data used for the tissue model.

## 🔧 Getting Started

1. Open the project in Unity (Recommended version 2021.3+ or 2022.3+).
2. Load the main Surgical Simulation Scene.
3. Press **Play**. Use the designated keys or VR/Mouse input to manipulate the `SurgicalTool` and interact with the liver model.

## 📄 License

MIT License

---
*Developed for advanced surgical simulation and interactive physically-based rendering research.*
