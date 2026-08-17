import yaml
with open("config/quadrotor_sim.yaml", "r") as f:
    config = yaml.safe_load(f)
config["plant"]["mass"] = 0.775
with open("config/quadrotor_sim.yaml", "w") as f:
    yaml.dump(config, f)
