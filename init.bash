host="vriufpr.ddns.net"

export UFR_CMDVEL="@new mqtt @coder msgpack @host $host @topic /pioneer/cmd_vel"
export UFR_SCAN="@new mqtt @coder msgpack @host $host @topic /pioneer/scan"
export UFR_ODOM="@new mqtt @coder msgpack @host $host @topic /pioneer/odom"
# export UFR_CAMERA  = "@new video @@new mqtt @@coder msgpack @@host $host @@topic /pioneer/camera"