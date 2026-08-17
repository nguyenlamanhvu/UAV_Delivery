import yaml
with open("config/quadrotor_sim.yaml", "r") as f:
    config = yaml.safe_load(f)
config["realtime_rate"] = 1.0
with open("config/quadrotor_sim.yaml", "w") as f:
    yaml.dump(config, f)
