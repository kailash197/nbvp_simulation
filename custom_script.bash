#!/usr/bin/bash

# /root/uav_ros_simulation/ros_packages/ardupilot_gazebo/scripts/shell_scripts.sh
# /root/uav_ros_simulation/ros_packages/uav_ros_stack/miscellaneous/shell_additions/shell_scripts.sh
# root@spectre:~# find . -name "shell_scripts.sh" -type f
# ./uav_ros_simulation/.gitman/uav_ros_stack_sparse/miscellaneous/shell_additions/shell_scripts.sh
# ./uav_ros_simulation/.gitman/ardupilot_gazebo/scripts/shell_scripts.sh

PWD=$(pwd)
cd /root
###################################
#!/bin/bash

# Find all shell_scripts.sh files
echo "Searching for shell_scripts.sh files..."
find . -name "shell_scripts.sh" -type f | while read -r file; do
    echo "Processing: $file"
    
    # Check if file contains "timeout 3s"
    if grep -q "timeout 3s" "$file"; then
        echo "  Found 'timeout 3s' in file"
        
        # Create backup
        cp "$file" "$file.bak"
        echo "  Backup created: $file.bak"
        
        # Only modify lines containing "timeout 3s"
        # This ensures we only add --preserve-status to the specific timeout commands
        sed -i '/timeout 3s/s/timeout /timeout --preserve-status /' "$file"
        
        echo "  Added --preserve-status to timeout 3s commands"
        
        # Show the modified lines
        echo "  Modified lines:"
        grep --color=always "timeout.*--preserve-status" "$file" || echo "  (no matches after modification)"
    else
        echo "  No 'timeout 3s' found, skipping"
    fi
    
    echo ""
done

echo "Done!"
######################
cd ${PWD}