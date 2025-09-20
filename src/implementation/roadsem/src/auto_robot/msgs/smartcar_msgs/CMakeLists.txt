cmake_minimum_required(VERSION 3.15.0)

project(smartcar_msgs)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        geometry_msgs
        sensor_msgs
        message_generation
        )

add_message_files(
        FILES
        State.msg
        Lane.msg
        ControlCommand.msg
        ControlCommandStamped.msg
        Waypoint.msg
        LaneArray.msg
        PointsImage.msg
        ProjectionMatrix.msg
        ImageObj.msg
        ImageObjects.msg
        ImageObjTracked.msg
        ImageObjRanged.msg
        ImageRect.msg
        ImageRectRanged.msg
        DetectedObject.msg
        DetectedObjectArray.msg
        TrafficLightResult.msg
        TrafficLight.msg
        TrafficLightResultArray.msg
        ImageLaneObjects.msg
        DiffSonic.msg
        CloudInterface.msg
        CrossLock.msg
        SonarInterface.msg
        UtmPoint.msg
        UtmWithPose.msg
        Gnss.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)



