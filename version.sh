#!/bin/bash

# Filename: generate_version.sh
# This script generates Git version information and writes it to a version.c file.

# Define the output file
OUTPUT_FILE="src/version.c"

# Get the Git version and status
GIT_VERSION=$(git describe --tags --always 2>/dev/null || echo "unknown")
GIT_STATUS=$(git status --porcelain 2>/dev/null)

# Determine if the working directory is clean
if [ -z "$GIT_STATUS" ]; then
    STATUS="clean"
else
    STATUS="dirty"
fi

# Create the version.c file
cat << EOF > $OUTPUT_FILE
// Generated version information
#include <stdio.h>

const char* git_version = "$GIT_VERSION";
const char* git_status = "$STATUS";

void print_version() {
    printf("Git Version: %s\\n", git_version);
    printf("Status: %s\\n", git_status);
}
EOF

echo "Generated $OUTPUT_FILE with Git version and status."

