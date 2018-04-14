mapboxgl.accessToken = 'pk.eyJ1IjoiZGVrc3ByaW1lIiwiYSI6ImNqOGsxb3dyYzA4b2wyeHBsdGx0aXdzeHYifQ.4vYgxeGhICEGWbC1552LsQ';

var ros_server_url = document.location.hostname + ":9090";
var ros = new ROSLIB.Ros();
var rosConnected = false;
var joy_msg1 = new ROSLIB.Message({
    linear: {
        x: 0.0,
        y: 0.0,
        z: 0.0
    },
    angular: {
        x: 0.0,
        y: 0.0,
        z: 0.0
    }
});
var joy_msg2 = new ROSLIB.Message({
    linear: {
        x: 0.0,
        y: 0.0,
        z: 0.0
    },
    angular: {
        x: 0.0,
        y: 0.0,
        z: 0.0
    }
});

var joystick_datas1 = new Array();

var joystick_datas2 = new Array();

var joystick_publisher1;

var joystick_publisher2;

var etanol_data;

var humidity_data;

var temp_data;

var metane_data;

var carbon_data;

var baro_data;

var data_metane = [56, 0, 78, 0, 0, 0, 0, 0];
var data_carbon = [0, 90, 0, 0, 0, 112, 0, 0];
var data_hum = [0, 0, 0, 45, 0, 0,0, 0];
var data_temp = [0, 87, 0, 0, 0, 75, 0, 0];
var data_baro = [0, 0, 66, 0, 0, 0, 0, 90];
var data_etanol = [0, 0, 34, 0, 0, 56, 0, 0];

var dps_metane=[];
var dps_carbon=[];
var dps_hum=[];
var dps_temp=[];
var dps_baro=[];
var dps_etanol=[];

//for (var i = 0, t = 100; i < t; i++) {
//    data.push(Math.round(Math.random() * 99))
//  };
//console.log(msg_f);


//var data_a;
//var data_split;
//var dps;

//var data_split = data_a.split(/,/);
//console.log(data_split);
//var dps = data_split;


//var dps = [78,22,34,12,50,63,77,98];
//var data = new Array();







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
    chart.render();
});

ros.on("close", function () {
    console.debug("Disconnected from ROS server");
    rosConnected = false;
});

// Create connection
ros.connect("ws://" + ros_server_url);

function initPublishers() {
    joystick_publisher1 = new ROSLIB.Topic({
        ros: ros,
        name: 'rover_joy/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });

    joystick_publisher2 = new ROSLIB.Topic({
        ros: ros,
        name: 'robotic_arm/data',
        messageType: 'geometry_msgs/Twist'
    });

    joystick_publisher1.publish(joy_msg1);
    joystick_publisher2.publish(joy_msg2);

}


function initSubscribers() {
    alert("sddghgsdffg")
    ////Define subscribers

    var humidty_listener = new ROSLIB.Topic({
        ros: ros,
        name: 'sensor_data', //dinlenecek topic adı
        messageType: 'sensor_msgs/string' //topicin mesaj tipi
    });

    var barometer_listener = new ROSLIB.Topic({
        ros: ros,
        name: 'sensor_data',
        messageType: 'sensor_msgs/string'
    });

    var temp_listener = new ROSLIB.Topic({
        ros: ros,
        name: 'sensor_data',
        messageType: 'sensor_msgs/string'
    });

    var carbon_listener = new ROSLIB.Topic({
        ros: ros,
        name: 'sensor_data',
        messageType: 'sensor_msgs/string'
    });

    var etanol_listener = new ROSLIB.Topic({
        ros: ros,
        name: 'sensor_data',
        messageType: 'sensor_msgs/string'
    });

    var metane_listener = new ROSLIB.Topic({
        ros: ros,
        name: 'sensor_data',
        messageType: 'sensor_msgs/string'
    });

    var sensor_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/sensor', //dinlenecek topic adı
        messageType: 'std_msgs/Int32MultiArray' //topicin mesaj tipi
    });






    //State
    //TODO Add Robostate /State topic
    //--Armed Status(True,False)
    //--Px4Mode(AUTO, OFFBOARD etc.)
    humidty_listener.subscribe(function (msg) {
        data_hum = msg.data;
    });
    barometer_listener.subscribe(function (msg) {
        data_baro = msg.data;
    });

    temp_listener.subscribe(function (msg) {
        data_temp = msg.data;
    });

    carbon_listener.subscribe(function (msg) {
        data_carbon = msg.data;

    });

    etanol_listener.subscribe(function (msg) {
        data_etanol = msg.data;


    });
    metane_listener.subscribe(function (msg) {
        console.log(msg);
        data_metane = msg.data;


    });

    sensor_listener.subscribe(function (msg) {
        console.log(msg);
        data = msg.data;
        console.log(data);
        //console.log(data);
        //console.log(data);
    });


    //function log(msg) {
    // $('#log').append(msg.toString();
    //}
    //sensor_listener.subscribe(function (msg) {

    //log(msg.data)


    //});







    //dps = msg_s.split(',');

    //data_split = data_a.split(/,/);

    //var dps = data_split;
    //buralar silinip diğerleri çift slaşa alınabilir
    // data_split = data_a.split(/,/);
    //console.log(data_split);
    //dps = data_split;
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
//Chart of X Sensor
var dps = [];

//dataPoints. 

var chart = new CanvasJS.Chart("chartContainer1", {
        title: {
            text: ""
        },
        axisX: {
            title: ""
        },
        axisY: {
            title: ""
        },
        toolTip: {
            shared: true
        },
        legend: {
            cursor: "pointer",
            itemclick: toggleDataSeries
        },
        data: [{
                type: "spline",
                name: "Metane",
                showInLegend: true,
                dataPoints: dps_metane
	},
            {
                type: "spline",
                name: "Barometer",
                showInLegend: true,
                dataPoints: dps_baro
	},
            {
                type: "spline",
                name: "Temperature",
                showInLegend: true,
                dataPoints: dps_temp
	},
            {
                type: "spline",
                name: "Carbon",
                showInLegend: true,
                dataPoints: dps_carbon
	},
            {
                type: "spline",
                name: "Etanol",
                showInLegend: true,
                dataPoints: dps_etanol
	},
            {
                type: "spline",
                name: "Humidity",
                showInLegend: true,
                dataPoints: dps_hum
	}],
    }


);
      function toggleDataSeries(e) {
        if (typeof (e.dataSeries.visible) === "undefined" || e.dataSeries.visible) {
            e.dataSeries.visible = false;
        } else {
            e.dataSeries.visible = true;
        }
        chart.render();
    }

//chart.render();
var xVal = 1;
var yVal = 0;
var updateInterval = 1000;

var updateChart = function () {




    dps_baro.push({
        x: xVal,
        y: +data_baro[yVal]
    });
    dps_carbon.push({
        x: xVal,
        y: +data_carbon[yVal]
    });
    dps_etanol.push({
        x: xVal,
        y: +data_etanol[yVal]
    });
    dps_hum.push({
        x: xVal,
        y: +data_hum[yVal]
    });
    dps_temp.push({
        x: xVal,
        y: +data_temp[yVal]
    });
    dps_metane.push({
        x: xVal,
        y: +data_metane[yVal]
    });



    yVal++;
    xVal++;

  
    chart.render();



    // update chart after specified time. 

};
updateChart(100);	
setInterval(function(){updateChart()}, updateInterval);





console.log(dps_baro);





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
    if (e.gamepad.index == 0) {
        for (j = 0; j < e.gamepad.axes.length; j++) {
            var axis_value = e.gamepad.axes[j].toFixed(3);
            joystick_datas1[j] = axis_value;
            console.log(joy_msg1);
            $("#joy" + (e.gamepad.index + 1) + "-axis-" + j + " .progress-bar").css("width", Math.abs(axis_value * 100) + "%");
            $("#joy" + (e.gamepad.index + 1) + "-axis-" + j + " .badge").text(axis_value);
        }
        joy_msg1.linear.x = -joystick_datas1[1];
        joy_msg1.angular.z = +joystick_datas1[2];
    }
    if (e.gamepad.index == 1) {
        for (j = 0; j < e.gamepad.axes.length; j++) {
            var axis_value = e.gamepad.axes[j].toFixed(3);
            joystick_datas2[j] = axis_value;
            console.log(joy_msg2);
            $("#joy" + (e.gamepad.index + 1) + "-axis-" + j + " .progress-bar").css("width", Math.abs(axis_value * 100) + "%");
            $("#joy" + (e.gamepad.index + 1) + "-axis-" + j + " .badge").text(axis_value);
        }
        joy_msg2.linear.x = +joystick_datas2[0];
        joy_msg2.linear.y = +joystick_datas2[1];
        joy_msg2.linear.z = +joystick_datas2[2];

        joy_msg2.angular.x = +joystick_datas2[3];
        joy_msg2.angular.y = +joystick_datas2[4];
        joy_msg2.angular.z = +joystick_datas2[5];
    }

    if (rosConnected) {
        joystick_publisher1.publish(joy_msg1);
        joystick_publisher2.publish(joy_msg2);
    }
});



//TODO Add marker arrays
//TODO Add list items and connect them to markers
//TODO Align the map and the info box - look at the columns
//TODO Draw linestrings in between the markers
//TODO Find a way to store array elements and save them into a file within the server
//when it is submitted
//TODO Make the waypoint markers draggable
