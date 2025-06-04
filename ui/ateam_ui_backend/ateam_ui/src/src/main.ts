import { createApp } from 'vue';
import vuetify from '../plugins/vuetify';
import App from './App.vue';


declare let Neutralino: any;
Neutralino.init();
Neutralino.events.on("windowClose", function(){Neutralino.app.exit()});

const app = createApp(App);
app.use(vuetify);

const vm = app.mount('#app');
