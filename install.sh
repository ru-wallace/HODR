#!/bin/bash

# Exit if not running as root
if [ "$(id -u)" -ne 0 ]; then
    echo "This script must be run as root. Please use sudo."
    exit 1
fi

#Check if packages are installed
REQUIRED_PACKAGES=("libglib2.0-dev" "libusb-dev")
NOT_INSTALLED=()
for package in "${REQUIRED_PACKAGES[@]}"; do
    if ! dpkg -l | grep -q "$package"; then
        echo "Package $package is not installed."
        NOT_INSTALLED+=("$package")
    fi
done

if [ ${#NOT_INSTALLED[@]} -ne 0 ]; then
    echo "The following required packages are not installed: ${NOT_INSTALLED[*]}"
    read -p "Do you want to install them with apt? (y/n): " answer
    if [[ "$answer" != "y" && "$answer" != "Y" ]]; then
        echo "Exiting installation script."
        exit 1
    fi
    echo "Installing missing packages..."
    apt update
    apt install -y "${NOT_INSTALLED[@]}"
else
    echo "All required packages are already installed."
fi

BASE_DIR=$(dirname -- "$( readlink -f -- "$0"; )";)


HOME_DIR="/home/$SUDO_USER"

MINIFORGE_DIR="$HOME_DIR/miniforge3"
if [ ! -d "$MINIFORGE_DIR" ]; then
    echo "Miniforge3 not found in $MINIFORGE_DIR. Please install it first."
    echo "Miniforge3 and installation instructions can be found at:"
    echo "          https://github.com/conda-forge/miniforge"
    echo "Exiting installation script."
    exit 1
fi

ANDOR_DIR="${MINIFORGE_DIR}/pkgs/andor2-sdk-2.*"

#Find the Andor2 SDK in the Miniforge3 directory
FOUND_DIR=$(find "$MINIFORGE_DIR/pkgs/" -type d -name "andor2-sdk-*" 2>/dev/null | head -n 1)

if [ -n "$FOUND_DIR" ]; then
    ANDOR_DIR="$FOUND_DIR"
    echo "Found Andor2 SDK in: $ANDOR_DIR"
    sudo -u $SUDO_USER echo "ANDOR_DIR=${ANDOR_DIR}" > "$BASE_DIR/config.mk"
else

    echo "Andor2 SDK not found. Please ensure it is installed in the Miniforge3 directory."
    echo "You can install it using:"
    echo "           conda install esrf-bcu::andor2-sdk"
    echo "If it is installed elsewhere, please update the ANDOR_DIR variable in this script."

    exit 1
fi


ln -s "$ANDOR_DIR/lib/libandor.so.2" "/usr/local/lib/libandor.so.2"
ldconfig


UDEV_RULE="SUBSYSTEM==\"usb\", ATTR{idVendor}==\"136e\", ATTR{idProduct}==\"0001\", GROUP=\"$SUDO_USER\", OWNER=\"$SUDO_USER\""

UDEV_RULE_FILE="/etc/udev/rules.d/99-hodr.rules"

echo "$UDEV_RULE" > "$UDEV_RULE_FILE"

echo "UDEV rule added to $UDEV_RULE_FILE"

udevadm control --reload-rules

echo "UDEV rules reloaded"

echo "Please unplug and replug the device for the changes to take effect."

ln -s "./hodr.service" "/etc/systemd/user/hodr.service"

echo "Building DBus headers using dbus-codegen..."
sudo -u $SUDO_USER make dbus > /dev/null 2>&1

if [ $? -ne 0 ]; then
    echo "Failed to build dbus. Please check the output for errors."
    echo "Ensure you have libglib2.0-dev installed."
    echo "You can install it using: sudo apt install libglib2.0-dev"
    exit 1
fi

echo "DBus headers built successfully."

echo "Building the project..."
sudo -u $SUDO_USER make 




