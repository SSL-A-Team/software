import 'vuetify/styles';
import { createVuetify } from 'vuetify';
import { aliases, mdi } from 'vuetify/iconsets/mdi'

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
        defaultSet: 'mdi',
        aliases,
        sets: {
            mdi
        }
    }
});
