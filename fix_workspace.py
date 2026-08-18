with open("gui/signal_workspace.cc", "r") as f:
    lines = f.readlines()

out = []
i = 0
while i < len(lines):
    if lines[i].strip() == "return;" and i + 2 < len(lines) and lines[i+1].strip() == "}" and lines[i+2].strip() == "":
        out.append(lines[i])
        i += 3
        continue
    out.append(lines[i])
    i += 1

with open("gui/signal_workspace.cc", "w") as f:
    f.writelines(out)

