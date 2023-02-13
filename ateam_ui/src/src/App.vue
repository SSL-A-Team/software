<template>
    <v-app>
        <v-app-bar app ref="Top Bar" color="#cbb26b" density="compact">
            <v-app-bar-title> ATeam UI </v-app-bar-title>
        </v-app-bar>
        <v-main>
            <v-container class="d-flex flex-row" ref="Main Components">
                <RefButtonsComponent/>
                <StatusComponent/>
                <FieldComponent/>
            </v-container>
        </v-main>
    </v-app>
</template>


<script lang="js">
import FieldComponent from './components/FieldComponent.vue'
import StatusComponent from './components/StatusComponent.vue'
import RefButtonsComponent from './components/RefButtonsComponent.vue'
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
                underlays: [],
                overlays: [],
                // Using Div A dimensions
                fieldDimensions: {
                    length: 12,
                    width: 9,
                    border: .7,
                    lineWidth: .01,
                    goalWidth: 1.8,
                    goalDepth: .18,
                    goalHeight: .16,
                    penaltyShort: 1.8,
                    penaltyLong: 3.6,
                    centerRadius: .5,
                    centerDiameter: 1,
                    goalFlat: .5,
                    floorLength: 13.4,
                    floorWidth: 10.4
                }
            },
            renderConfig: {
                angle: 0,
                scale: 300, // Pixels per meter (in the rendering canvas)
                factor: .25 // Field Scaling Factor (applied to the canvas when it's added to the UI)
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
    StatusComponent,
    RefButtonsComponent
}
}

</script>
