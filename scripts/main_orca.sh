gnome-terminal -- bash -c "./orca.sh"

echo "Waiting for initialization (sleep 15s)..."
sleep 15
source ../devel/setup.bash
rosrun orca_planner goal_publisher.py
