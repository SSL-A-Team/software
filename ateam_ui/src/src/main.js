import { createApp } from 'vue';
import vuetify from './plugins/vuetify';
import VueKonva from 'vue3-konva';
import App from './App.vue';
import 'roslib/build/roslib';

const app = createApp(App);

app.use(vuetify);
app.use(VueKonva);

const vm = app.mount('#app');

// Configure Default Ball Location
vm.state.ball = {
    x: 0,
    y: 0,
    visible: true
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
            x: (70*(i+xoffset)) - (70*(numVisible-1)/2),
            y: (120*yoffset) - 60,
            rotation: team=="blue" ? 180 : 0,
            visible: (i<=numVisible-1),
            status: {
                connected: true,
                kicker: false,
                battery: 100,
                name: null,
                message: null
            }
        })
    };
    yoffset += 1;
}

// Configure Default Overlays
vm.state.overlays.push({
    type: 'line',
    x: 0,
    y: 0,
    fill: "red"
});


var ros = new window.ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});


var listener = new ROSLIB.Topic({
    ros: ros,
    name: '/listener',
    messageType: 'std_msgs/String'
});

listener.subscribe(function(msg) {console.log(msg)})

var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/cmd_vel',
    messageType : 'geometry_msgs/Twist'
});
var twist = new ROSLIB.Message({
    linear : {
        x : 0.1,
        y : 0.2,
        z : 0.3
    },
    angular : {
        x : -0.1,
        y : -0.2,
        z : -0.3
    }
});
cmdVel.publish(twist);
