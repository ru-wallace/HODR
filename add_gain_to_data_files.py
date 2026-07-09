import pathlib


hodr_data_dir = pathlib.Path(__file__).parent.parent / "hodr_data"



for data_file in hodr_data_dir.glob("*.csv"):
    print(f"Processing file: {data_file}")
    lines = []
    with open(data_file, "r") as f:
        lines = f.readlines()
    

    for line in lines:
        cols = line.strip().split(",")
        # if str(len(cols)) not in len_dict:
        #     len_dict[str(len(cols))] = 0
        #     print(f"Added new key to len_dict: {len(cols)}")
        #     print("Found in file:", data_file)
        # len_dict[str(len(cols))] += 1
        if len(cols) != 1027:
            continue
        
        # insert pre-amp gain value as 3rd column (index 2)
        pre_amp_gain = 1.7  # default value
        cols.insert(2, str(pre_amp_gain))
        new_line = ",".join(cols) + "\n"
        lines[lines.index(line)] = new_line
    
    with open(data_file, "w") as f:
        f.writelines(lines)

