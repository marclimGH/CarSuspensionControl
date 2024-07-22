## Vehicle Suspension Modeling with Modelica

https://github.com/user-attachments/assets/6e18f32e-0519-4f9e-9e56-349e14bb0cf2

### Overview
The `SuspensionSystem` package provides an advanced simulation platform for vehicle suspension systems using Modelica. It is designed for automotive engineers and researchers focusing on dynamics analysis and system design. This package features detailed models of suspension components and road interactions, along with sophisticated control strategies for both passive and active damping systems. All examples within the package are visualized through 3D animations, enhancing the comprehension of dynamic interactions and mechanical responses.

### Components
#### RoadProfile
Simulates varying road surfaces as a low-pass filtered white noise, representing road roughness dynamically affecting the vehicle's suspension system.

#### Wheel
This model encapsulates the wheel assembly with detailed properties including mass and tire elasticity. It is crucial for evaluating the dynamic response of the entire suspension system to simulated road textures.

#### QuarterCarModel
Extends the basic quarter-car setup to include simulations of both passive and active damping mechanisms under identical road and vehicle conditions. This allows for direct comparisons of different suspension strategies, focusing on impacts on ride comfort and stability.

#### Dampers
Features models for both passive dampers and advanced active dampers. The active damper model includes a dynamic transfer function from the control input to the damping force output, with a response time of 10-30 ms representing solenoid-actuated dampers.

#### Controllers
- **Acceleration-Driven Damping (ADD)**: Adjusts damper settings based on acceleration feedback to improve comfort and handling on rough surfaces.
- **Mixed SkyHook-ADD Control**: Integrates traditional SkyHook control with ADD. SkyHook is optimized for low frequencies while ADD excels at high frequencies. A frequency selector dynamically splits control between the two methods, adapting to varying driving conditions for optimal performance.

### Getting Started
Clone the repository to explore detailed 3D animated examples in the `Examples` package, featuring simulations for both passive and active suspension setups using identical models and parameters. This setup facilitates a fair comparison between the two systems under similar conditions. Users can modify simulation parameters to tailor the analysis to specific road conditions and vehicle dynamics.

### Technical Specifications
- **Modelica Version**: 4.0.0, compatible with various Modelica-based simulation tools like OpenModelica.
- **Visualization and Animation**: Includes detailed settings for 3D visualization of component movement and interaction, providing a rich, intuitive understanding of the suspension system's behavior.

