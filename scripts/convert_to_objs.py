import os

def parse_file_content(file_content):
    sections = {'F_after:': [], 'V_after:': [], 'F_before:': [], 'V_before:': []}
    current_section = None

    for line in file_content.splitlines():
        if line in sections:
            current_section = line
            continue
        if current_section and line.strip():
            sections[current_section].append(line.strip())
    return sections

def write_obj(file_path, vertices, faces):
    with open(file_path, 'w') as obj_file:
        for v in vertices:
            obj_file.write(f"v {v}\n")
        for f in faces:
            # OBJ files are 1-indexed, so we need to add 1 to each index
            indices = [str(int(index) + 1) for index in f.split()]
            obj_file.write(f"f {' '.join(indices)}\n")

def process_files(directory):
    for file_name in os.listdir(directory):
        if file_name.startswith("local_atlas_") and file_name.endswith(".txt"):
            with open(os.path.join(directory, file_name), 'r') as file:
                content = file.read()
                sections = parse_file_content(content)

                # Prepare to write OBJ files
                base_name = file_name.rsplit('.', 1)[0]
                before_obj_path = os.path.join(directory, f"{base_name}_before.obj")
                after_obj_path = os.path.join(directory, f"{base_name}_after.obj")

                write_obj(before_obj_path, sections['V_before:'], sections['F_before:'])
                write_obj(after_obj_path, sections['V_after:'], sections['F_after:'])

# Replace '/path/to/your/files' with the actual directory containing your files
process_files('./')
