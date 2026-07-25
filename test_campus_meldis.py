import pydrake.all
from pydrake.all import (
    DiagramBuilder,
    DrakeVisualizer,
    Parser,
    Simulator,
)

def main():
    print("Building Diagram...")
    builder = DiagramBuilder()
    
    plant, scene_graph = pydrake.multibody.plant.AddMultibodyPlantSceneGraph(
        builder, time_step=1e-3
    )
    
    parser = Parser(plant)
    
    print("Loading campus SDF...")
    try:
        campus_model_idx = parser.AddModels("models/campus.sdf")[0]
        print("Successfully loaded the campus model.")
    except Exception as e:
        print(f"Error loading model: {e}")
        return
        
    plant.Finalize()
    
    # Add DrakeVisualizer (LCM) instead of Meshcat
    # This will stream the geometry to Meldis natively
    DrakeVisualizer.AddToBuilder(builder, scene_graph)
    
    diagram = builder.Build()
    simulator = Simulator(diagram)
    
    print("\n=========================================================")
    print("Simulation running! Open a SECOND terminal and run:")
    print("  conda activate pydrake_quick_test")
    print("  python3 -m pydrake.visualization.meldis")
    print("=========================================================\n")
    print("Press Ctrl+C in this terminal to stop the simulation.")
    
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(1000000.0)

if __name__ == "__main__":
    main()
