# Create the hooks directory if it doesn't exist
mkdir -p install/aerial_robot_core/share/aerial_robot_core/environment

# Create the hook script
cat > install/aerial_robot_core/share/aerial_robot_core/environment/env_hooks.sh << 'EOF'
export RCUTILS_COLORIZED_OUTPUT=1
EOF