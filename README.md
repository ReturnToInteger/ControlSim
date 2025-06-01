# ControlSim

This project started as a Formula Student Car simulation in MATLAB. I revisited this to port it to C++.

## Features

- Simulate car movement with custom parameters
- Navigate through a map with obstacles, currently has cones as obstacles.
- Keyboard navigation
- Visualize environment using SFML graphics
- Use either Matlab .mat files, an iterable class of cones, or your own custom class
- Modular map reader (`IMapReader`), vehicle controller (`IControllerLogic`)

## Screenshot
![Simulation Screenshot](screenshot.png)

## Requirements

- Visual Studio
 > Tested on Visual Studio 2022; older versions should work if they support C++17.  
- C++17
- SFML 2.6.2 (needed if you want graphics)

Optional:
 - MATLAB (needed for reading `mat` files using `MatlabMapReader`)

## Getting Started

Clone the repository:

```bash
   git clone https://github.com/yourusername/yourproject.git
   ```

Open the .sln file in Visual Studio and follow the setup instructions below.


## How to Run / Setup
1. **MATLAB support (optional):**  
   - Add MATLAB’s `extern\include` and `extern\lib\<YOUR_OP_SYSTEM>` header folders to your project include paths.  (Already set up for some common install paths)
   - Add the MATLAB `bin` folder to your system `PATH`.  
   - Matlab libraries needed are already set up.
2. **SFML setup:**  
   - Install SFML and add its `include` and `lib` folders to your project settings.  (Set up for `C:\Libraries\SFML\`)
   - Copy SFML DLLs to your build output folder or add their location to `PATH`.

3. **Build and Run:**  
   - Open the `.sln` file in Visual Studio.  
   - Build the project (Debug or Release).  
   - Run the executable.

## MATLAB Support

By default, the project includes support for reading `.mat` files using `MatlabReader`.

If you don't have MATLAB installed or don't need this feature:

1. Open the project in Visual Studio.
2. Right-click `MatlabMapReader.cpp` → **Properties**.
3. Set **Excluded From Build** to `Yes` (for both Debug and Release).
4. In `ControlSim.cpp` switch to `ManualMapReader` or another map reader class.

