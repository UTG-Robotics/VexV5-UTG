with open("data.csv", "r") as f:
    for line in f:
        line = line.strip()
        if "Kv" in line:
            line = line.replace("Kv: ", "")
            line = line.split("|")
            print(f"{line[1]},{line[0]}")