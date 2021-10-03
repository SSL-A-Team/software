import { createApp } from 'vue';
import vuetify from './plugins/vuetify';
import VueKonva from 'vue3-konva';
import App from './App.vue';

const app = createApp(App);

app.use(vuetify);
app.use(VueKonva);

const vm = app.mount('#app');

// Configer Default Ball Location
vm.state.ball = {
    x: 560/2,
    y: 360/2,
    visible: true
}

// Configure Default Robot Layout
var yoffset = 0;
for (var team in vm.state.teams) {
    var xoffset = 0;
    for (var i = 0; i < 16; i++) {
        vm.state.teams[team].robots.push({
            id: i,
            team: team,
            x: 60 + (70*(i+xoffset)),
            y: 60 + (70*yoffset),
            visible: (i<=6)
        })

        if (i!=0 && ((i+1) % 8) == 0) {
            xoffset-=8;
            yoffset+=1;
        }
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


// Configure Field Dimensions
vm.state.fieldDimensions = {
    length: 9,
    width: 6,
    border: .7,
    lineWidth: .01,
    goalWidth: 1,
    goalDepth: .18,
    goalHeight: .16,
    penaltyShort: 1,
    penaltyLong: 2,
    centerRadius: .5,
    centerDiameter: 1,
    goalFlat: .5,
    floorLength: 10.4,
    floorWidth: 7.4
};
