#!/bin/bash

# Loop through every file ending in .csv
for file in *.csv; do
    # Check if the file exists to avoid errors in empty directories
    [ -e "$file" ] || continue
    
    echo "Processing $file..."
    python3 csvgraph.py "$file"
done

echo "All files processed!"