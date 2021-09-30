import 'vuetify/styles'
import { createApp } from 'vue'
import { createVuetify } from 'vuetify'
import colors from 'vuetify/lib/util/colors'
import App from './App.vue'


const vuetify = createVuetify({theme: {defaultTheme: 'dark'}});
const app = createApp(App).use(vuetify).mount('#app');

console.log(Object.getOwnPropertyNames(vuetify))
