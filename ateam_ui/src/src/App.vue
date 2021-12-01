<template>
    <v-app>
        <v-app-bar ref="Top Bar" color="red" density="compact">
            <v-app-bar-title> ATeam UI </v-app-bar-title>
        </v-app-bar>
        <v-main>
            <v-container class="d-flex flex-row" ref="Main Components">
                <StatusComponent/>
                <FieldComponent/>
            </v-container>
        </v-main>
    </v-app>
</template>


<script lang="js">
import FieldComponent from './components/FieldComponent.vue'
import StatusComponent from './components/StatusComponent.vue'
import { provide } from 'vue'

export default {
    data() {
        return {
            state: {
                sim: true,
                team: "yellow",
                ball: {
                    pose: {
                        position: {}
                    },
                    twist: {},
                    accel: {}
                },
                teams: {
                    blue: {
                        name: "",
                        color: "blue",
                        defending: +1,
                        robots: []
                    },
                    yellow: {
                        name: "",
                        color:  "yellow",
                        defending: -1,
                        robots: [],
                    }
                },
                overlays: [],
                fieldDimensions: {
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
    components: {
        FieldComponent,
        StatusComponent
    }
}

</script>
