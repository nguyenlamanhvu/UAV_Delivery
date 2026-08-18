with open("gui/signal_workspace.cc", "r") as f:
    content = f.read()

content = content.replace("    return;\n  // Old implementation ignored", "    return;\n  }\n  // Old implementation ignored")

with open("gui/signal_workspace.cc", "w") as f:
    f.write(content)
