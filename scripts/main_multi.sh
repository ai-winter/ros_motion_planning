gnome-terminal -- bash -c "./multi.sh"

echo "Waiting for initialization (sleep 15s)..."
sleep 15
source ../devel/setup.bash
rosrun sim_env goal_publisher.py
