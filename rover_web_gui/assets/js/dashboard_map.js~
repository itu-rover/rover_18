mapboxgl.accessToken = 'pk.eyJ1IjoiZGVrc3ByaW1lIiwiYSI6ImNqOGsxb3dyYzA4b2wyeHBsdGx0aXdzeHYifQ.4vYgxeGhICEGWbC1552LsQ';

var ros_server_url = document.location.hostname + ":9090";
var ros = new ROSLIB.Ros();
var rosConnected = false;
var joy_msg = new ROSLIB.Message({
    linear : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    },
    angular : {
      x : -0.0,
      y : -0.0,
      z : -0.0
    }
  });
var joystick_datas = {
    axis_0 : 0.0,
    axis_1 : 0.0,
    axis_2 : 0.0,
    axis_3 : 0.0,
    axis_4 : 0.0,
    axis_5 : 0.0
}
var joystick_publisher;
window.gamepad = new Gamepad();
var focused = false;
var monument = [-110.791941, 38.406320];
var map = new mapboxgl.Map({
    container: 'map',
    style: 'mapbox://styles/mapbox/satellite-streets-v9',
    center: [-110.791941, 38.406320],
    zoom: 18
});
map.addControl(new mapboxgl.NavigationControl(), 'bottom-right');
map.doubleClickZoom.disable();

ros.on("connection", function () {
    console.debug("Connected to ROS server");
    rosConnected = true;
    initSubscribers();
    initPublishers();
});

ros.on("close", function () {
    console.debug("Disconnected from ROS server");
    rosConnected = false;
});

// Create connection
ros.connect("ws://" + ros_server_url);

function initPublishers(){
       joystick_publisher = new ROSLIB.Topic({
       ros : ros,
       name : 'rover_joy/cmd_vel',
       messageType : 'geometry_msgs/Twist'
    });
}
function initSubscribers() {
    ////Define subscribers

    var humidty_listener = new ROSLIB.Topic({
        ros: ros,
        name: 'humidity_topic', //dinlenecek topic adı
        messageType: 'sensor_msgs/string' //topicin mesaj tipi
    });

    var barometer_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/state',
        messageType: 'mavros_msgs/State'
    });

    var temp_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/global_position/global',
        messageType: 'sensor_msgs/NavSatFix'
    });

    var carbon_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/global_position/compass_hdg',
        messageType: 'std_msgs/Float64'
    });

    var etanol_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/local_position/odom',
        messageType: 'nav_msgs/Odometry'
    });


    //State
    //TODO Add Robostate /State topic
    //--Armed Status(True,False)
    //--Px4Mode(AUTO, OFFBOARD etc.)
    humidty_listener.subscribe(function (msg) {
        //msg.string şeklinde mesajı alabilirsin
    });
    barometer_listener.subscribe(function (msg) {
    });

    temp_listener.subscribe(function (msg) {
    });

    carbon_listener.subscribe(function (msg) {

    });
    
    etanol_listener.subscribe(function (msg) {

    });
    

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


//GeoJson object for drone marker
var drone = {
    "type": "Point",
    "coordinates": monument
};

// GeoJSON object to hold our measurement features
var geojson = {
    "type": "FeatureCollection",
    "features": []
};

//heading value fo drone marker
var direction;

//sync drone position 
function setDronePos() {
    map.getSource('drone').setData(drone);

    map.setLayoutProperty('drone', 'icon-rotate', direction);

    if (focused === true) {
        map.setCenter(drone.coordinates);
    }
}


// Used to draw a line between points
//Added comments for Taha
var linestring = {
    "type": "Feature",
    "geometry": {
        "type": "LineString",
        "coordinates": []
    }
};


// create DOM element for the marker
var el = document.createElement('div');
el.id = 'marker';
var click_counter = 0;
// create the marker
map.on('click', function (e) {
    new mapboxgl.Marker(el)
        .setLngLat(e.lngLat)
        .setPopup(popup) // sets a popup on this marker
        .addTo(map);
    click_counter = click_counter + 1;
    //'Waypoint #: ' + '<br />' +
    // e.lngLat is the longitude, latitude geographical position of the event
    //JSON.stringify(e.lngLat);
});

map.on('load', function () {
    // add the GeoJSON above to a new vector tile source
    if (rosConnected) {
        map.addSource('drone', {
            type: 'geojson',
            data: drone
        });

        map.setCenter(drone.coordinates);

        map.addLayer({
            "id": "drone-glow-strong",
            "type": "circle",
            "source": "drone",
            "paint": {
                "circle-radius": 18,
                "circle-color": "#fff",
                "circle-opacity": 0.4
            }
        });

        map.addLayer({
            "id": "drone-glow",
            "type": "circle",
            "source": "drone",
            "paint": {
                "circle-radius": 40,
                "circle-color": "#fff",
                "circle-opacity": 0.1
            }
        });

        map.addLayer({
            "id": "drone",
            "type": "symbol",
            "source": "drone",
            "layout": {
                "icon-image": "airport-15",
                "icon-rotation-alignment": "map"
            }
        });

        window.setInterval(setDronePos, 10);
    }
});

jQuery("#go-btn").click(function () {
    map.setCenter(drone.coordinates);
});

jQuery("#focus-btn").click(function () { //focuses on the marker
    if (focused === true) {
        focused = false;
        jQuery(this).removeClass("disabled");
    } else {
        focused = true;
        jQuery(this).addClass("disabled");
    }
});

//--joystick stuff comes here--//

// Attach it to the window so it can be inspected at the console.
gamepad.bind(Gamepad.Event.CONNECTED, function (device) {
    console.log('Connected', device);
});

gamepad.bind(Gamepad.Event.DISCONNECTED, function (device) {
    console.log('Disonnected', device);

});

if (!gamepad.init()) {
    alert('Your browser does not support gamepads, get the latest Google Chrome or Firefox.');
}

gamepad.bind(Gamepad.Event.AXIS_CHANGED, function (e) {
    for (j = 0; j < e.gamepad.axes.length; j++) {
        var axis_value = e.gamepad.axes[j].toFixed(3);
        joystick_datas[j] = axis_value;
        console.log("#joy" + (e.gamepad.index + 1) + "-axis-" + j + " .progress-bar");
        $("#joy" + (e.gamepad.index + 1) + "-axis-" + j + " .progress-bar").css("width", Math.abs(axis_value * 100) + "%");
        $("#joy" + (e.gamepad.index + 1) + "-axis-" + j + " .badge").text(axis_value);
    }
    joystick_datas[1] = -joy_msg.linear.y;
    joystick_datas[0] = joy_msg.angular.z;
    
    if(rosConnected){ 
        joystick_publisher.publish(joy_msg);
    }
});



//TODO Add marker arrays
//TODO Add list items and connect them to markers
//TODO Align the map and the info box - look at the columns
//TODO Draw linestrings in between the markers
//TODO Find a way to store array elements and save them into a file within the server
//when it is submitted
//TODO Make the waypoint markers draggable