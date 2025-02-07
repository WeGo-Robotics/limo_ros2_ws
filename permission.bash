#!/bin/bash

# Get the current directory path
current_path=$(pwd)

# Change directory to the parent directory of the folders containing the Python files
cd "$current_path"

sudo apt install libgoogle-glog-dev -y

# Build work space
colcon build --symlink-install

sudo find . -name "*.py" -type f -exec chmod 777 {} \;

# Change permissions of all Python files in subdirectories
SHELL_TYPE="$(basename "$SHELL")"
echo "--------------------"  
echo "Shell Type : $SHELL_TYPE"
if [ "$SHELL_TYPE" == "bash" ]; then
  # Bash shell
  if ! grep -q "source $current_path/install/setup.bash" ~/.zshrc; then
    echo "source $current_path/install/setup.bash" >> ~/.bashrc
    echo "Added 'source $current_path/install/setup.bash' to ~/.bashrc"
    else
    echo "'source $current_path/install/setup.bash' is already created in the bashrc file."
  fi
elif [ "$SHELL_TYPE" == "zsh" ]; then
  # Zsh shell
	if ! grep -q "source $current_path/install/setup.zsh" ~/.zshrc; then
    echo "source $current_path/install/setup.zsh" >> ~/.zshrc
    echo "Added 'source $current_path/install/setup.zsh' to ~/.zshrc"
    else
    echo "'source $current_path/install/setup.zsh' is already created in the zshrc file."
  fi
else
  echo "Unsupported shell type: $SHELL_TYPE"
  exit 1
fi

# Setup Completed
echo "Setup completed."
echo "--------------------"