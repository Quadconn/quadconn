SERVICES=(
    "quadconn_startup.target"
    "zero_motors.service"
    "motor_interface.service"
    "controls.service"
    "lidar.service"
    "controller_udp.service"
    "diagnostics.service"
)

echo "--- Quadconn Service Status ---"
for service in "${SERVICES[@]}"; do
    STATUS=$(systemctl is-active $service)
    if [ "$STATUS" == "active" ]; then
        echo -e "[ \e[32mOK\e[0m ] $service is $STATUS"
    elif [ "$STATUS" == "activating" ]; then
        echo -e "[ \e[33mWAIT\e[0m ] $service is $STATUS"
    else
        echo -e "[ \e[31mFAIL\e[0m ] $service is $STATUS"
    fi
done
echo "-------------------------------"