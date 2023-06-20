import { createApp } from 'vue';
import vuetify from './plugins/vuetify';
import App from './App.vue';
import { AppState } from '@/state';


declare let Neutralino: any;
Neutralino.init();
Neutralino.events.on("windowClose", function(){Neutralino.app.exit()});

const app = createApp(App);

app.use(vuetify);

import VueKonva from 'vue-konva';
app.use(VueKonva);

const vm = app.mount('#app');
