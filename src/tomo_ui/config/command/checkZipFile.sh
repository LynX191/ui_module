#!/bin/bash

# Get the path to the ZIP file from the first argument
zipfile=$1
typefile=$2

# Check if the ZIP file exists
if [ ! -f "$zipfile" ]; then
    echo "ZIP file does not exist."
    exit 1
fi

# Create a temporary directory to extract the ZIP file
tmpdir=$(mktemp -d)

# Unzip the file into the temporary directory
unzip -q "$zipfile" -d "$tmpdir"

# Check if the directory install/share/tomo_ui exists in the extracted contents
if [ -d "$tmpdir/install/share/$typefile" ]; then
    echo "Folder $typefile exists"
    exit 1
else
    echo "Folder $typefile does not exist"
    exit 0
fi

# Clean up by removing the temporary directory
rm -rf "$tmpdir"
