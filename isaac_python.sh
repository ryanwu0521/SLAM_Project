CURR_DIR="$(pwd)"
FILE_PATH="$CURR_DIR/$1"

if [[ -z "$ISAAC_PATH" ]]; then
    echo "Setting ISAAC_PATH to default install path"
    ISAAC_PATH="$HOME/.local/share/ov/pkg/isaac_sim-2023.1.1"
fi

cd $ISAAC_PATH
# echo "$ISAAC_PATH"
# echo "$FILE_PATH"
./python.sh "$FILE_PATH"