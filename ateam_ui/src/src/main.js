import { createApp } from 'vue';
import vuetify from './plugins/vuetify';
import VueKonva from 'vue3-konva';
import App from './App.vue';

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
            visible: (i<=numVisible-1)
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
