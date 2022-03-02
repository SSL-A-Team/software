import 'vuetify/styles';
import { createVuetify } from 'vuetify';


export default createVuetify({
    theme:{
        defaultTheme: 'dark',
        options: {
            customProperties: true
        },
        themes: {
            dark: {
                field: 'green',
                lines: 'white'
            }
        }
    },
    icons: {
        iconfont: 'mdi'
    }
});
