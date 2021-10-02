import { createApp } from 'vue';
import vuetify from './plugins/vuetify';
import VueKonva from 'vue3-konva';
import App from './App.vue';

const app = createApp(App);

app.use(vuetify);
app.use(VueKonva);

const vm = app.mount('#app');
console.log("app mounted");

vm.state.robots[0].y=200;
