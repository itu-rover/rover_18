//Change the port below if you changed in launch file
var ros_server_url = document.location.hostname + ":9090";
var ros = new ROSLIB.Ros();
var rosConnected = false;

// Set event listener for ROS Connect event
ros.on("connection", function() {
    console.debug("Connected to ROS server");
    rosConnected = true;
    initSubscribers();
});

// Create connection
ros.connect("ws://" + ros_server_url);


function initSubscribers() {
    ////Define subscribers

    var battery_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/battery',
        messageType: 'sensor_msgs/BatteryState'
    });

    var state_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/state',
        messageType: 'mavros_msgs/State'
    });

    var global_position_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/global_position/global',
        messageType: 'sensor_msgs/NavSatFix'
    });

    var compass_hdg_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/global_position/compass_hdg',
        messageType: 'std_msgs/Float64'
    });

    var local_odom_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/local_position/odom',
        messageType: 'nav_msgs/Odometry'
    });

    var imu_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/imu/data',
        messageType: 'sensor_msgs/Imu'
    });

    //State
    //TODO Add Robostate /State topic
    //--Armed Status(True,False)
    //--Px4Mode(AUTO, OFFBOARD etc.)
    state_listener.subscribe(function(msg) {
        var armed_status = msg.armed.toString();
        var armed_placeholder = '';
        if (armed_status == 'false') {
            armed_placeholder = 'NO';
        } else {
            armed_placeholder = 'YES';
        }
        jQuery("#txtPx4Mode").html(msg.mode.toString());
        jQuery("#txtArmedStatus").html(armed_placeholder.toString())
    });
    //Battery
    //--Voltage(V)
    //--Percentage(%)
    //--Percentage bar design loader
    //--Current(A)
    //--Consumed(mAh)
    battery_listener.subscribe(function(msg) {
        jQuery("#txtBatteryVoltage").html(msg.voltage.toFixed(2) + " V");
        jQuery("#txtBatteryPercentage").html(msg.percentage.toFixed(3) * 100 + "%");
        jQuery("#progBatteryPercent").css("width", msg.percentage.toFixed(3) * 100 + "%");
        jQuery("#txtBatteryCurrent").html(msg.current.toFixed(2) + " A");
        jQuery("#txtBatteryConsumed").html((8000 * (1 - msg.percentage)).toFixed(1) + " mAh"); //TODO: make battery capacity parametric or get it from the topic message
    });
    //Global
    //--Latitude (degrees)
    //--Longitude (degrees)
    //--Altitude (m)AMSL
    //TODO add artificial horizon
    global_position_listener.subscribe(function(msg) {
        jQuery("#txtLatitude").html(msg.latitude + "°");
        jQuery("#txtLongitude").html(msg.longitude + "°");
        jQuery("#txtGlobalAltitude").html(msg.altitude.toFixed(2) + " m");
        drone.coordinates[1] = msg.latitude;
        drone.coordinates[0] = msg.longitude;
    });
    //--Compass heading
    compass_hdg_listener.subscribe(function(msg) {
        jQuery("#txtCompassHeading").html(msg.data);
        direction = msg.data;
    });
    //Local
    //--Position x, y, z (m)
    //--Orientation x, y, z, w (angle)(TODO convert to euler angles AND to degrees bc
    //--Velocity x, y, z, (m/s)

    //-Mission
    //--TODO Mission type (topic to be determined)
    //--TODO add percentage bar design animation here as well
    //--TODO Mission Percentage (topic to be determined)
    //--TODO Horizontal Distance (topic to be determined)
    //--TODO Waypoints (/mavros/Waypoints)
    //--TODO Total Distance(squarecube(x,y,z))
    //--
    //--TODO Function
    ///
}


/*function loadURDF() {
    urdf_viewer = new ROS3D.Viewer({
        divID: "divURDF",
        width: jQuery("#divURDF").width(),
        height: jQuery("#divURDF").width() * 0.8,
        antialias: true,
        background: "#E0F0F5"
    });
    urdf_viewer.addObject(new ROS3D.Grid({ color: "#a0a0a0" }));
    console.log(urdf_viewer.height);

    tf_client = new ROSLIB.TFClient({
        ros: ros,
        angularThres: 0.01,
        transThres: 0.01,
        rate: 10.0,
        fixedFrame: "/base_link"
    });

    urdf_client = new ROS3D.UrdfClient({
        ros: ros,
        tfClient: tf_client,
        path: document.location + "/packages/",
        rootObject: urdf_viewer.scene

    });

    jQuery(window).on('resize', function() {
        urdf_viewer.resize(jQuery("#divURDF").width(), jQuery("#divURDF").width() * 0.8);
    });

}*/
