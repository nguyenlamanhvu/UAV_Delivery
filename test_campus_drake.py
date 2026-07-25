import pydrake.all
from pydrake.all import (
    DiagramBuilder,
    MeshcatVisualizer,
    MultibodyPlant,
    Parser,
    Simulator,
    StartMeshcat,
)

def main():
    print("Starting Meshcat...")
    meshcat = StartMeshcat()
    print(f"Meshcat URL: {meshcat.web_url()}")
    
    builder = DiagramBuilder()
    
    plant, scene_graph = pydrake.multibody.plant.AddMultibodyPlantSceneGraph(
        builder, time_step=1e-3
    )
    
    parser = Parser(plant)
    
    print("Loading campus SDF...")
    try:
        # Load the SDF we just created
        campus_model_idx = parser.AddModels("models/campus.sdf")[0]
        
        print("Successfully loaded the campus model.")
    except Exception as e:
        print(f"Error loading model: {e}")
        return
        
    plant.Finalize()
    
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    
    diagram = builder.Build()
    simulator = Simulator(diagram)
    
    # Run the simulation for a very long time so Meshcat stays open
    print("Simulating... Open the Meshcat URL in your browser.")
    print("Press Ctrl+C in this terminal when you are done to close it.")
    simulator.set_target_realtime_rate(1.0)
    
    # Simulates for ~11 days straight
    simulator.AdvanceTo(1000000.0)

if __name__ == "__main__":
    main()
