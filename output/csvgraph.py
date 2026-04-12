import matplotlib
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import pandas as pd
import sys
import os

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 csvgraph.py file1.csv file2.csv file3.csv")
        return

    # Define your custom labels in order
    custom_labels = ["Mesh", "Fully Connected", "2D Folded Torus"]
    
    plt.figure(figsize=(10, 6))
    
    # We use enumerate to keep track of which file number we are on (0, 1, 2...)
    for i, filename in enumerate(sys.argv[1:]):
        if not os.path.exists(filename):
            print(f"Warning: File '{filename}' not found. Skipping.")
            continue

        try:
            df = pd.read_csv(filename, header=None)
            x = df.iloc[:, 0]
            y = df.iloc[:, 1]
            
            # Pick the custom label if it exists, otherwise use the filename
            if i < len(custom_labels):
                current_label = custom_labels[i]
            else:
                current_label = os.path.basename(filename)
            
            plt.plot(x, y, marker='o', linestyle='-', label=current_label)
            print(f"Plotting {filename} as '{current_label}'")

        except Exception as e:
            print(f"Error processing {filename}: {e}")

    plt.title("Local - Uniform Traffic")
    plt.xlabel("Injection Rate (flits/cycle/chip)")
    plt.ylabel("Average Latency (cycles)")
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.7)

    output_fn = "Local_Uniform.png"
    plt.savefig(output_fn)
    print(f"\nSuccess! Plot saved as: {output_fn}")

if __name__ == "__main__":
    main()


#python3 csvgraph.py local-chiplet-uniform.csv local-fc-uniform.csv local-torus-uniform.csv