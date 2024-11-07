import os
import subprocess
import sys

# Path to the proto files
PROTO_PATH = os.path.join(os.getcwd(), 'proto')

# Path to the batch file (e.g., protoc.bat)
PROTOC_BAT_PATH = os.path.join(os.getcwd(), '.pio', 'libdeps', 'esp32dev', 'Nanopb', 'generator', 'protoc.bat')

# Ensure the proto directory exists
if not os.path.isdir(PROTO_PATH):
    print(f"Error: Directory '{PROTO_PATH}' does not exist.")
    sys.exit(1)

# Check if the batch file exists
if not os.path.isfile(PROTOC_BAT_PATH):
    print(f"Error: Batch file '{PROTOC_BAT_PATH}' does not exist.")
    sys.exit(1)

# Gather all .proto files in the proto directory
proto_files = [os.path.join(PROTO_PATH, f) for f in os.listdir(PROTO_PATH) if f.endswith('.proto')]
if not proto_files:
    print(f"No .proto files found in '{PROTO_PATH}'.")
    sys.exit(1)

# Run the batch file for each .proto file
for proto in proto_files:
    # Including --proto_path to point to the proto directory
    cmd = [PROTOC_BAT_PATH, f'--proto_path={PROTO_PATH}', '--nanopb_out=.', proto]
    print(f"Running: {' '.join(cmd)}")

    try:
        # Run the batch file and capture output
        result = subprocess.run(cmd, check=True, capture_output=True, text=True, shell=True)
        print(result.stdout)  # Print standard output
    except subprocess.CalledProcessError as e:
        print(f"Error generating files for {proto}:", e.stderr)
