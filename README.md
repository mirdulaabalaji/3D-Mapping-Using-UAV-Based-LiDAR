# Fixed-Wing UAV LiDAR Mapping Simulation in MATLAB

This project is a MATLAB-based simulation of a fixed-wing Unmanned Aerial Vehicle (UAV) equipped with a LiDAR sensor for 3D terrain mapping. The primary objective is to create a testbed for validating UAV guidance algorithms and sensor data acquisition pipelines.

The project is presented in two distinct versions:
1.  **Code 1 (Synthetic Environment):** A foundational proof-of-concept that operates in a fully controlled, procedurally generated world.
2.  **Code 2 (Real-World Application):** A practical, advanced version that is modified to load real-world Digital Elevation Model (DEM) data and perform a realistic survey mission.

## 🚀 Features

* **UAV Kinematics:** Simulates a fixed-wing UAV following a 3-DOF kinematic model.
* **Waypoint Guidance:** Implements a P-controller for yaw to follow a pre-defined list of waypoints.
* **LiDAR Sensor Model:** Simulates a 2D scanning LiDAR to generate a 3D point cloud.
* **Environment Generation:**
    * (Code 1) Procedurally generates a "forest" environment with rolling hills and conical "trees."
    * (Code 2) Loads, parses, and converts real-world GeoTIFF DEM files.
* **Terrain Following:** (Code 2) Implements a critical Above Ground Level (AGL) terrain-following logic.
* **Visualization:**
    * Real-time 3D animation of the UAV, terrain, and live-mapped points.
    * Final height-colored 3D scatter plot of the generated point cloud.
* **Data Export:** Saves the final `[X, Y, Z]` point cloud data to a `uav_lidar_map.csv` file.

## Code 1 vs. Code 2: Project Evolution

The project's main value is in its evolution from a simple concept to a practical tool.

### Code 1: Synthetic "Forest" Environment

This code serves as the baseline proof-of-concept. Its primary goal is to validate that the core components (UAV guidance, sensor model, data logging) work together.

* **Terrain:** Procedurally generated using `sin` and `cos` functions for rolling hills, with a `for` loop adding conical "trees" as obstacles.
* **Flight Path:** A simple, hard-coded 6-point S-shaped path.
* **Flight Logic:** The UAV flies at a constant **Mean Sea Level (MSL)** altitude. This is a simple but unrealistic model.
* **Data Handling:** Uses simple (but inefficient) dynamic array concatenation to build the `mapPoints` array.

### Code 2: Real-World DEM Application

This code is a significant upgrade, modified for a practical, real-world mission rehearsal.

* **Terrain:** Loads a real-world Digital Elevation Model (DEM) from a GeoTIFF (`.tif`) file using the **Mapping Toolbox**. It performs coordinate conversion from Lat/Lon to a local meter-based grid.
* **Flight Path:** Replaces the static path with a **procedural lawnmower pattern generator**. This automatically calculates a multi-line survey path to cover a rectangular area.
* **Flight Logic:** This is the most critical change. It implements a dynamic **Above Ground Level (AGL)** terrain-following model. The UAV queries the DEM in real-time to adjust its altitude, maintaining a constant height *above the ground*.
* **Data Handling:** The code is optimized for performance. It **pre-allocates** a large array for `mapPoints` and uses a counter, which is orders of magnitude faster and necessary for a large-scale simulation.

### Summary of Key Differences

| Feature | Code 1 (Synthetic) | Code 2 (Real-World) |
| :--- | :--- | :--- |
| **Terrain Source** | Procedurally Generated (Sine waves, Cones) | Real-World DEM File (`.tif`) |
| **Flight Logic** | Fixed **MSL** Altitude | Dynamic **AGL** (Terrain-Following) |
| **Flight Path** | Static, hard-coded S-path | Procedural "Lawnmower" Grid |
| **Dependencies** | MATLAB Core | MATLAB + **Mapping Toolbox** |
| **Data Handling** | Dynamic Concatenation (Inefficient) | Pre-allocation (Efficient) |
| **Primary Goal** | Validate core guidance & sensor logic | Rehearse a practical, large-scale survey mission |

## ⚙️ Requirements

* **MATLAB** (e.g., R2021b or newer)
* **Mapping Toolbox** (This is **required** for Code 2 to run the `readgeoraster` function)

## ▶️ How to Run

### Code 1 (Synthetic Environment)

1.  Open the `code_1_synthetic.m` script in MATLAB.
2.  Run the script.
3.  The animation will run, and the final plots will be generated.
4.  A `uav_lidar_map.csv` file will be saved to the same directory.

### Code 2 (Real-World DEM)

1.  Place your Digital Elevation Model (DEM) file (e.g., `myDEM.tif`) in the same directory as the script.
2.  Open the `code_2_real_world.m` script.
3.  **Crucial Step:** Change the `dem_filename` variable (line 62) to match the name of your file:
    ```matlab
    dem_filename = 'myDEM.tif'; % 
    ```
4.  Ensure the **Mapping Toolbox** is installed.
5.  Run the script. The simulation may take longer due to the larger area and longer simulation time.
6.  The animation and final plots will be generated, and a `uav_lidar_map.csv` file will be saved.

## Outputs

1.  **Real-time Animation Figure:** A 3D plot showing the UAV's progress, the terrain, and the live-mapped points.

<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/c32414fd-165c-46b2-8ff6-d285db94793b" alt="Point Cloud of Code 1" width="600">
      <br>
      <em>Fig 1(a). Real-time animation for Code 1 - Generated Synthetic Terrain</em>
    </td>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/f8329735-fd50-41d1-aa93-c4dac09d82e6" alt="Point Cloud of Code 1" width="600">
      <br>
      <em>Fig 1(b). Real-time animation for Code 2 - GeoTIFF Terrain from DEM</em>
    </td>
  </tr>
</table>

2.  **Final Point Cloud Figure:** A high-resolution, height-colored `scatter3` plot of the complete, aggregated point cloud.

<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/430d9b14-c9e5-455a-91f1-cbf13822761b" alt="Point Cloud of Code 1" width="600">
      <br>
      <em>Fig 2(a). Final point cloud for Code 1 - Generated Synthetic Terrain</em>
    </td>
    <td align="center">
      <img  src="https://github.com/user-attachments/assets/0cabe9a3-c506-4a29-bbb0-9b5cba6179fc" alt="Point Cloud of Code 1" width="600">
      <br>
      <em>Fig 2(b). Final point cloud for Code 2 - GeoTIFF Terrain from DEM</em>
    </td>
  </tr>
</table>
   
3.  **Data File:** `uav_lidar_map.csv` containing the `[X, Y, Z]` coordinates of every point in the generated map.


Inspired by [Airborne LiDAR, archaeology, and the ancient Maya landscape at Caracol, Belize](https://www.sciencedirect.com/science/article/abs/pii/S0305440310003286?via%3Dihub)
