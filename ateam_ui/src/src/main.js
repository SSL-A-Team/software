import { createApp } from 'vue';
import vuetify from './plugins/vuetify';
import VueKonva from 'vue3-konva';
import App from './App.vue';
import 'roslib/build/roslib';


Neutralino.init();
Neutralino.events.on("windowClose", function(){Neutralino.app.exit()});

const app = createApp(App);

app.use(vuetify);
app.use(VueKonva);

const vm = app.mount('#app');

// Configure Default Ball Location
vm.state.ball = {
    pose: {
        position: {
            x: 0,
            y: 0
        }
    },
    twist: {},
    accel: {},
    visible: true,
    publisher: null
}

// Configure Default Robot Layout
const numVisible = 16;
var yoffset = 0;
for (var team in vm.state.teams) {
    var xoffset = 0;
    for (var i = 0; i < 16; i++) {
        vm.state.teams[team].robots.push({
            id: i,
            team: team,
            pose: {
                position: {
                    x: vm.renderConfig.scale * 0.23 * (i+xoffset - ((numVisible-1)/2)),
                    y: vm.renderConfig.scale * ((0.4*yoffset) - 0.2)
                },
                orientation: {}
            },
            twist: {},
            accel: {},
            rotation: team=="blue" ? 180 : 0,
            visible: (i<=numVisible-1),
            status: {
                connected: true,
                kicker: false,
                battery: 100,
                name: null,
                message: null,
                publisher: null
            }
        })
    };
    yoffset += 1;
}

// Configure Default Overlays
vm.state.underlays = [];
vm.state.overlays = [];


// Configure ROS
var ros = new window.ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', function() {
    console.log('Connected to ROS server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to ROS server: ', error);
});

ros.on('close', function() {
    console.log('Connection to ROS server closed.');
    Neutralino.app.exit();
});

// Set up ball subscribers and publishers
var ballTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/ball',
    messageType: 'ateam_msgs/msg/BallState'
});

ballTopic.subscribe(function(msg) {
    vm.state.ball.pose.position = msg.pose.position;
});

//TODO: add publisher for moving sim ball


// Set up robot subscribers and publishers
function getRobotCallback(team, id){
    return function(msg) {
        var robot = vm.state.teams[team].robots[id];
        robot.pose = msg.pose;

        // Account for the XY switch and rad 2 degrees
        robot.rotation = 2*Math.acos(robot.pose.orientation.z)*180/3.14 - 90;
    };
};

for (var team in vm.state.teams) {
    for (var i = 0; i < 16; i++) {
        var robotTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/' + team + '_team/robot' + i,
            messageType: 'ateam_msgs/msg/RobotState'
        });

        robotTopic.subscribe(getRobotCallback(team, i));

        //TODO: add publisher for moving sim robots
    }
}

// Set up overlay subscriber
var overlayTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/overlay',
    messageType: 'ateam_msgs/msg/Overlay'
});

overlayTopic.subscribe(function(msg) {
    var id = msg.ns+"/"+msg.name;
    var location = (msg.depth==0) ? vm.state.underlays : vm.state.overlays;
    switch(msg.command) {
        // REPLACE
        case 0:
            location[id] = msg;
            break;
        // EDIT
        case 1:
            //TODO: Not sure if this command is necessary, will implement later if it is
            // Might need to handle moving overlay between z-depths
            location[id] = msg;
            break;
        // REMOVE
        case 2:
            delete vm.state.overlays[id];
            break;
    }
});

//TODO: add all other pub/subs (field dimensions, referee, etc)
