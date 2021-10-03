<template>
    <v-app>
        <v-app-bar color="red" density="compact">
            <v-app-bar-title> ATeam UI </v-app-bar-title>
        </v-app-bar>
        <v-main>
            <FieldComponent/>
        </v-main>
    </v-app>
</template>


<script lang="js">
import FieldComponent from './components/FieldComponent.vue'
import { defineComponent, provide, reactive, ref, computed } from 'vue'

export default {
    data() {
        return {
            state: {
                sim: true,
                team: "yellow",
                ball: {},
                teams: {
                    blue: {
                        name: "",
                        color: "blue",
                        robots: []
                    },
                    yellow: {
                        name: "",
                        color:  "yellow",
                        robots: []
                    }
                },
                overlays: [],
                fieldDimensions: {
                    width: 2,
                    length: 1
                }
            },
            renderConfig: {
                angle: 0,
                scale: 300 // Pixels per meter
            }
        }
    },
    provide() {
        return {
            state: this.state,
            renderConfig: this.renderConfig
        }
    },
    components: {FieldComponent}
}

</script>
