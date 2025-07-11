import 'vuetify/styles';
import { createVuetify, ThemeDefinition } from 'vuetify';
import { aliases, mdi } from 'vuetify/iconsets/mdi'

const aTeamTheme: ThemeDefinition = {
    dark: true,
    colors: {
        'field': '#008000',
        'lines': '#ffffff',
        'ball': '#ffa500',
        'blue-team-field': '0000ff',
        'yellow-team-field': 'ffff00',
        'blue-team-text': '#ffffff',
        'yellow-team-text': '#000000',
        'status-shell-color': '00008b',
        'status-controlled-robot': '008000',
        'status-visibility': '0000ff',
        'status-warn': 'ffff00',
        'status-error': 'ff0000',
        'dot-pattern-pink': 'ff1493',
        'dot-pattern-green': '7cfc00',
        'ateam-color': '#cbb26b'
    },
    variables: {
        name: 'A-Team Theme'
    }
}

export default createVuetify({
    theme:{
        defaultTheme: 'aTeamTheme',
        options: {
            customProperties: true
        },
        themes: {
            aTeamTheme
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
